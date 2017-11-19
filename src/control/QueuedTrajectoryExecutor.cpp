#include <chrono>
#include <aikido/control/QueuedTrajectoryExecutor.hpp>

namespace aikido {
namespace control {

//==============================================================================
QueuedTrajectoryExecutor::QueuedTrajectoryExecutor(
    std::unique_ptr<TrajectoryExecutor> executor)
  : mExecutor{std::move(executor)}, mInProgress{false}, mMutex{}
{
  // Do nothing
}

//==============================================================================
QueuedTrajectoryExecutor::~QueuedTrajectoryExecutor()
{
  // Do nothing.
}

//==============================================================================
std::future<void> QueuedTrajectoryExecutor::execute(
    trajectory::TrajectoryPtr traj)
{
  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    // Queue the trajectory and promise that will need to be set
    mTrajectoryQueue.push(std::move(traj));
    mPromiseQueue.emplace(new std::promise<void>());
    return mPromiseQueue.back()->get_future();
  }
}

//==============================================================================
void QueuedTrajectoryExecutor::step()
{
  mExecutor->step();

  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning

  // If a trajectory was executing, check if it has finished
  if (mInProgress)
  {
    // Return if the trajectory is still executing
    if (mFuture.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
      return;

    // The promise corresponding to the trajectory that just finished must be
    // at the front of its queue.
    auto promise = mPromiseQueue.front();
    mPromiseQueue.pop();

    // Propagate the future's value or exception to the caller
    try
    {
      mFuture.get();
      promise->set_value();
    }
    catch (const std::exception& e)
    {
      promise->set_exception(std::current_exception());
    }

    mInProgress = false;
  }

  // No trajectory currently executing, execute a trajectory from the queue
  if (!mTrajectoryQueue.empty())
  {
    trajectory::TrajectoryPtr traj = mTrajectoryQueue.front();
    mTrajectoryQueue.pop();

    mFuture = mExecutor->execute(std::move(traj));
    mInProgress = true;
  }
}

} // namespace control
} // namespace aikido
