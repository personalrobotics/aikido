#include <aikido/ros_control/RosTrajectoryExecutor.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <chrono>
#include <thread>

namespace aikido{

namespace ros_control{

RosTrajectoryExecutor::RosTrajectoryExecutor( 
    ::dart::dynamics::SkeletonPtr skeleton, std::chrono::milliseconds period,
    std::string serverName, ros::NodeHandle node, double trajTimeStep):

    mSkeleton(std::move(skeleton)),
    mSpinMutex(),
    mRunning(true),
    mPromise(nullptr),
    mPeriod(period),
    mServerName(std::move(serverName)),
    mNode(std::move(node)),
    mTrajClientPtr(new TrajectoryClient(serverName)),
    mTrajTimeStep(trajTimeStep)
{
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;

    if (!mSkeleton)
        throw std::invalid_argument("Skeleton is null.");

    if (mPeriod.count() <= 0)
        throw std::invalid_argument("Period must be positive.");

    mThread = std::thread(&RosTrajectoryExecutor::spin, this);

    // Set up action client (needs to be done AFTER previous line I think ????? )
    mTrajClientPtr.reset(new TrajectoryClient(serverName));

    mTrajClientPtr->waitForServer(); //Here
}

RosTrajectoryExecutor::~RosTrajectoryExecutor()
{
    {
        std::lock_guard<std::mutex> lockSpin(mSpinMutex);
        mRunning = false;
    }

    mCv.notify_one();

    mThread.join();
}

// ASSUMES STATESPACE IS RN!!!!!!

std::future<void> RosTrajectoryExecutor::execute(
    trajectory::TrajectoryPtr _traj)
{
    using statespace::dart::MetaSkeletonStateSpace;

    if(!_traj)
        throw std::invalid_argument("Traj is null.");

    auto space = std::dynamic_pointer_cast<
        MetaSkeletonStateSpace>(_traj->getStateSpace());

    if(!space)
        throw std::invalid_argument("Trajectory does not operate in this Executor's"
            " MetaSkeletonStateSpace.");

    auto metaSkeleton = space->getMetaSkeleton();

    control_msgs::FollowJointTrajectoryGoal traj_goal;

    std::unique_lock<std::mutex> skeleton_lock(mSkeleton->getMutex());
    for (auto dof : metaSkeleton->getDofs())
    {
        auto name = dof->getName();
        auto dof_in_skeleton = mSkeleton->getDof(name);

        if(!dof_in_skeleton){
            std::stringstream msgl
            msg << "_traj contrains dof [" << name 
            << "], which is not in mSkeleton.";
            
            throw std::invalid_argument(msg.str());
        }

        // Appropriate place to do this?
        traj_goal.trajectory.joint_names.push_back(name);
    }

    skeleton_lock.unlock();

    {
        std::lock_guard<std::mutex> lockSpin(mSpinMutex);

        if(mTraj)
            throw std::runtime_error("Another trajectory in execution.");

        mPromise.reset(new std::promise<void>());
        mTraj = _traj;
    }

    mCv.notify_one();

    // Convert aikido trajectory to a set of trajectory_msgs/JointTrajectoryPoint
    // and put in traj_goal.trajectory

    auto space = std::dynamic_pointer_cast<
        MetaSkeletonStateSpace>(mTraj->getStateSpace());
    size_t n_dims = space->getDimension(); 

    double start = mTraj->getStartTime();
    double duration = mTraj->getDuration();

    int time_steps = (int)(duration/mTrajTimeStep);

    for(int i=0; i <= time_steps; i++)
    {
        double time_val = start + i*mTrajTimeStep;
        trajectory_msgs::JointTrajectoryPoint jtpoint;
        auto state = space->createState();
        mTraj->evaluate(time_val, state);

        // Assign underlying values for Rn
        // ASSUMPTION - ORDER OF RN VALS IS SAME AS DOF ORDER?
        Eigen::VectorXd state_vect = state.getValue();
        double* state_values = state_vect.data();

        // Is this correct??????
        jtpoint.positions.assign(state_values, state_values+n_dims);
        jtpoint.time_from_start = time_val - start;

        traj_goal.trajectory.points.push_back(jtpoint);
    }

    if(duration - i*mTrajTimeStep > std::numeric_limits<double>::epsilon())
    {
        // Not a perfect division of time-steps
        auto state = space->createState();
        mTraj->evaluate(time_val, state);

        Eigen::VectorXd state_vect = state.getValue();
        double* state_values = state_vect.data();

        jtpoint.positions.assign(state_values, state_values+n_dims);
        jtpoint.time_from_start = time_val - start;

        traj_goal.trajectory.points.push_back(jtpoint);
    }

    // Send goal to server

    mTrajClientPtr -> sendGoal(traj_goal);

    //Wait for Result - need a TIMEOUT parameter
    double timeout = 30.0;
    bool finished_before_timeout = mTrajClientPtr -> waitForResult(ros::Duration(timeout));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState client_state = mTrajClientPtr->getState();
        ROS_INFO("Action finished: %s",client_state.toString().c_str());
    }
    else{
        ROS_INFO("Action did not finish before the time out.");
    }

    return mPromise->get_future();

}



void RosTrajectoryExecutor::spin()
{
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using statespace::dart::MetaSkeletonStateSpace;
    using dart::common::make_unique;
    using std::chrono::system_clock;

    system_clock::time_point startTime;
    bool trajInExecution = false; 

    while(true)
    {
        // Terminate the thread if mRunning is false.
        std::unique_lock<std::mutex> lockSpin(mSpinMutex);
        mCv.wait(lockSpin, [&] {
            // Reset startTime at the beginning of mTraj's execution.
            if (!trajInExecution && this->mTraj)
            {
                startTime = system_clock::now();
                trajInExecution = true;
            }
      
        return this->mTraj || !mRunning;  
        }); 

        if (!mRunning)
        {
            if (this->mTraj){
                mPromise->set_exception(std::make_exception_ptr(
                std::runtime_error("Trajectory terminated while in execution.")));
                this->mTraj.reset();
            }
            break;
        }
    
        // Can't do static here because MetaSkeletonStateSpace inherits 
        // CartesianProduct which inherits virtual StateSpace 
        auto space = std::dynamic_pointer_cast<
            MetaSkeletonStateSpace>(mTraj->getStateSpace());
        auto metaSkeleton = space->getMetaSkeleton();
        auto state = space->createState();

        // Get current time on mTraj.
        system_clock::time_point const now = system_clock::now();
        double t = duration_cast<duration<double> >(now - startTime).count();

        mTraj->evaluate(t, state);

        // Lock the skeleton, set state.
        std::unique_lock<std::mutex> skeleton_lock(mSkeleton->getMutex());
        space->setState(state);
        skeleton_lock.unlock();

        // Check if trajectory has completed.
        bool const is_done = (t >= mTraj->getEndTime());
        if (is_done) {
            mTraj.reset();
            mPromise->set_value();
            trajInExecution = false;
        } 

        std::this_thread::sleep_until(now + mPeriod);
    }
}

}
}