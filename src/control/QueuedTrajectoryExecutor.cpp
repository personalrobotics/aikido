#include "aikido/control/QueuedTrajectoryExecutor.hpp"
#include <chrono>

namespace aikido {
namespace control {

//==============================================================================
QueuedTrajectoryExecutor::QueuedTrajectoryExecutor(
    std::shared_ptr<TrajectoryExecutor> executor)
  : mExecutor{std::move(executor)}, mInProgress{false}, mMutex{}
{
  if (!mExecutor)
    throw std::invalid_argument("Executor is null.");
}

//==============================================================================
QueuedTrajectoryExecutor::~QueuedTrajectoryExecutor()
{
  // Do nothing.
}

//==============================================================================
void QueuedTrajectoryExecutor::validate(const trajectory::Trajectory* traj)
{
  mExecutor->validate(traj);
}

//==============================================================================
std::future<void> QueuedTrajectoryExecutor::execute(
    const trajectory::ConstTrajectoryPtr& traj)
{
  validate(traj.get());

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
void QueuedTrajectoryExecutor::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  mExecutor->step(timepoint);

  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning

  // If a trajectory was executing, check if it has finished
  if (mInProgress)
  {
    // Return if the trajectory is still executing
    if (mFuture.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
      return;

    mInProgress = false;

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
      abort();
    }
  }

  // No trajectory currently executing, execute a trajectory from the queue
  if (!mTrajectoryQueue.empty())
  {
    trajectory::ConstTrajectoryPtr traj = mTrajectoryQueue.front();
    mTrajectoryQueue.pop();

    mFuture = mExecutor->execute(std::move(traj));
    mInProgress = true;
  }
}

//==============================================================================
void QueuedTrajectoryExecutor::abort()
{
  std::lock_guard<std::mutex> lock(mMutex);
  DART_UNUSED(lock); // Suppress unused variable warning

  std::exception_ptr abort
      = std::make_exception_ptr(std::runtime_error("Trajectory aborted."));

  if (mInProgress)
  {
    mExecutor->abort();

    // Set our own exception, since abort may not be supported
    auto promise = mPromiseQueue.front();
    mPromiseQueue.pop();
    promise->set_exception(abort);

    mInProgress = false;
  }

  // Trajectory and promise queue are now the same length
  while (!mPromiseQueue.empty())
  {
    auto promise = mPromiseQueue.front();
    mPromiseQueue.pop();
    promise->set_exception(abort);

    mTrajectoryQueue.pop();
  }

  mFuture = std::future<void>();
}

} // namespace control
} // namespace aikido
