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
  const ::dart::dynamics::SkeletonPtr& _skeleton)
: mSkeleton(_skeleton)
, mMutex()
, mRunning(true)
, mPromise(nullptr)
, mTraj(nullptr)
{
  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");

  mThread = std::thread(&KinematicSimulationTrajectoryExecutor::spin, this);
}

//=============================================================================
KinematicSimulationTrajectoryExecutor::~KinematicSimulationTrajectoryExecutor()
{
  std::unique_lock<std::mutex> lock(mMutex);
  mRunning = false;
  lock.unlock();
  
  mThread.join();
}
//=============================================================================
std::future<TrajectoryResultPtr> KinematicSimulationTrajectoryExecutor::execute(
  trajectory::TrajectoryPtr _traj)
{

  if (!_traj)
    throw std::invalid_argument("Traj is null.");

  std::unique_lock<std::mutex> lock(mMutex);

  if (mTraj)
    throw std::runtime_error("Another trajectory in execution.");
  lock.unlock();

  using statespace::dart::MetaSkeletonStateSpace;

  auto space = std::dynamic_pointer_cast<
    MetaSkeletonStateSpace>(_traj->getStateSpace());

  if (!space)
    throw std::invalid_argument("Trajectory does not operate in thie executor's"
      "MetaSkeletonStateSpace.");

  auto metaSkeleton = space->getMetaSkeleton();

  for (auto dof : metaSkeleton->getDofs())
  {
    auto name = dof->getName();
    auto dof_in_skeleton = mSkeleton->getDof(name);

    if (!dof_in_skeleton)
      throw std::invalid_argument("_traj contains dof not in mSkeleton.");
  }

  lock.lock();
  mPromise.reset(new std::promise<TrajectoryResultPtr>());
  mTraj = _traj;
  lock.unlock();

  return mPromise->get_future();

}

//=============================================================================
void KinematicSimulationTrajectoryExecutor::spin()
{
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using statespace::dart::MetaSkeletonStateSpace;
  using dart::common::make_unique;

  static std::chrono::milliseconds const period(2);

  while(true)
  {
    // Terminate if mRunning is false.
    std::unique_lock<std::mutex> lock(mMutex);
    bool running = mRunning;
    lock.unlock();

    if (!running)
      break;
    
    // Wait until mTraj is set.
    if (!mTraj) 
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    std::chrono::system_clock::time_point const startTime
      = std::chrono::system_clock::now();

    auto space = std::dynamic_pointer_cast<
      MetaSkeletonStateSpace>(mTraj->getStateSpace());

    auto metaSkeleton = space->getMetaSkeleton();
    auto state = space->createState();

    // loop to set trajectory.
    while(true) {

      // Get current time on mTraj.
      std::chrono::system_clock::time_point const now
        = std::chrono::system_clock::now();
      double t = duration_cast<duration<double> >(now - startTime).count();
      double trajStartTime = mTraj->getStartTime();
      double trajCurrentTime = trajStartTime + t; 

      mTraj->evaluate(trajCurrentTime, state);

      // Lock the skeleton, set state.
      std::unique_lock<std::mutex> skeleton_lock(mSkeleton->getMutex());
      space->setState(state);
      skeleton_lock.unlock();

      // Break the loop if trajectory has completed.
      bool const is_done = (trajCurrentTime >= mTraj->getEndTime());
      if (is_done) {
        // Set future, reset trajectory.
        lock.lock();
        mTraj.reset();
        mPromise->set_value(make_unique<TrajectoryResult>());
        lock.unlock();
        break;
      } 
      std::this_thread::sleep_until(now + period);
    }

  }

}

}
}
