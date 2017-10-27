#include <ros/ros.h>
#include <aikido/control/TrajectoryExecutor.hpp>

namespace aikido {
namespace control {

//==============================================================================
TrajectoryExecutor::TrajectoryExecutor()
  : mRunning(false), mReturnWhenEmpty(false)
{
  // Do nothing.
}

//==============================================================================
TrajectoryExecutor::~TrajectoryExecutor()
{
  mRunning.store(false);
  if (mThread.joinable())
    mThread.join();
}

//==============================================================================
void TrajectoryExecutor::queue(trajectory::TrajectoryPtr traj)
{
  std::lock_guard<std::mutex> lock(mTrajectoryQueueMutex);
  mTrajectoryQueue.push(std::move(traj));
}

//==============================================================================
void TrajectoryExecutor::setExecuteFromQueue(bool flag)
{
  const bool isRunning = mRunning.exchange(flag);
  if (flag && !isRunning)
    mThread = std::thread(&TrajectoryExecutor::executeFromQueue, this);
}

//==============================================================================
void TrajectoryExecutor::executeFromQueue()
{
  ::ros::Rate rate(30);

  while (mRunning.load() && ::ros::ok())
  {
    trajectory::TrajectoryPtr traj;

    std::unique_lock<std::mutex> lock(mTrajectoryQueueMutex);
    if (!mTrajectoryQueue.empty())
    {
      traj = mTrajectoryQueue.front();
      mTrajectoryQueue.pop();
    }
    else
    {
      if (mReturnWhenEmpty.load())
        mRunning.store(false);
    }
    lock.unlock();

    // TODO: we should verify that this trajectory can actually be run, i.e.
    // that the start of this trajectory is close to the current state
    // See https://github.com/personalrobotics/rewd_controllers/issues/6

    if (traj)
      execute(traj).wait();

    rate.sleep();
  }

  mRunning.store(false);
}

//==============================================================================
void TrajectoryExecutor::executeAllFromQueue()
{
  mReturnWhenEmpty.store(true);
  if (mThread.joinable())
    mThread.join();
}

} // namespace control
} // namespace aikido
