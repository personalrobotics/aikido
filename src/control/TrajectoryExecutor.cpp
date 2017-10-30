#include <aikido/control/TrajectoryExecutor.hpp>
#include <chrono>

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
  // TODO: can this be rewritten with aikido::common::ExecutorThread?

  const auto period = std::chrono::milliseconds(100);
  while (mRunning.load())
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

    std::this_thread::sleep_until(std::chrono::steady_clock::now() + period);
  }

  mRunning.store(false);
}

//==============================================================================
void TrajectoryExecutor::executeAllFromQueue()
{
  mReturnWhenEmpty.store(true);
  if (mThread.joinable())
    mThread.join();
  mReturnWhenEmpty.store(false);
}

} // namespace control
} // namespace aikido
