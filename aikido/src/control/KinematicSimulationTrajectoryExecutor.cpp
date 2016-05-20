#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <chrono>
#include <thread>
#include <dart/common/StlHelpers.h>

#include <aikido/statespace/SO2.hpp>
using aikido::statespace::SO2;

namespace aikido{
namespace control{

//=============================================================================
KinematicSimulationTrajectoryExecutor::KinematicSimulationTrajectoryExecutor(
  ::dart::dynamics::SkeletonPtr _skeleton,
  std::chrono::milliseconds _period)
: mSkeleton(std::move(_skeleton))
, mSpinMutex()
, mRunning(true)
, mPromise(nullptr)
, mTraj(nullptr)
, mPeriod(_period)
{
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::milliseconds;

  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");

  if (mPeriod.count() <= 0)
    throw std::invalid_argument("Period must be positive.");

  mThread = std::thread(&KinematicSimulationTrajectoryExecutor::spin, this);
}

//=============================================================================
KinematicSimulationTrajectoryExecutor::~KinematicSimulationTrajectoryExecutor()
{
  {
    std::lock_guard<std::mutex> lockSpin(mSpinMutex);
    mRunning = false;
  }
  mCv.notify_one();
  
  mThread.join();
}
//=============================================================================
std::future<void> KinematicSimulationTrajectoryExecutor::execute(
  trajectory::TrajectoryPtr _traj)
{
  using statespace::dart::MetaSkeletonStateSpace;

  if (!_traj)
    throw std::invalid_argument("Traj is null.");

  auto space = std::dynamic_pointer_cast<
    MetaSkeletonStateSpace>(_traj->getStateSpace());

  if (!space)
    throw std::invalid_argument("Trajectory does not operate in this Executor's"
      " MetaSkeletonStateSpace.");

  auto metaSkeleton = space->getMetaSkeleton();

  // Check if metaSkeleton contains Dofs only in mSkeleton.
  std::unique_lock<std::mutex> skeleton_lock(mSkeleton->getMutex());
  for (auto dof : metaSkeleton->getDofs())
  {
    auto name = dof->getName();    
    auto dof_in_skeleton = mSkeleton->getDof(name);

    if (!dof_in_skeleton){
      std::stringstream msg;
      msg << "_traj contrains dof [" << name 
      << "], which is not in mSkeleton.";

      throw std::invalid_argument(msg.str());
    }
  }
  skeleton_lock.unlock();

  {
    std::lock_guard<std::mutex> lockSpin(mSpinMutex);

    if (mTraj)
      throw std::runtime_error("Another trajectory in execution.");

    mPromise.reset(new std::promise<void>());
    mTraj = _traj;  
  }
  mCv.notify_one();
  
  return mPromise->get_future();

}

//=============================================================================
void KinematicSimulationTrajectoryExecutor::spin()
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
