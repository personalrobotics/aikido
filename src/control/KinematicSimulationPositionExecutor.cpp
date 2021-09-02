#include "aikido/control/KinematicSimulationPositionExecutor.hpp"

#include <dart/common/Console.hpp>

#include "aikido/common/memory.hpp"

namespace aikido {
namespace control {

//==============================================================================
KinematicSimulationPositionExecutor::KinematicSimulationPositionExecutor(
    ::dart::dynamics::SkeletonPtr skeleton)
  : PositionExecutor(skeletonToJointNames(skeleton))
  , mSkeleton{std::move(skeleton)}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
  , mExecutionStartTime{std::chrono::system_clock::now()}
{
  if (!mSkeleton)
  {
    stop();
    throw std::invalid_argument("Skeleton is null.");
  }
}

//==============================================================================
KinematicSimulationPositionExecutor::~KinematicSimulationPositionExecutor()
{
  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
    {
      mInProgress = false;
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Command canceled.")));
    }
    stop();
  }
}

//==============================================================================
std::future<int> KinematicSimulationPositionExecutor::execute(
    const std::vector<double> command,
    const std::chrono::duration<double>& timeout)
{

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning
    auto promise = std::promise<int>();

    if (command.size() != mJoints.size())
    {
      promise.set_exception(std::make_exception_ptr(
          std::runtime_error("DOF of command does not match DOF of joints.")));
      return promise.get_future();
    }

    // Overwrite previous position command
    if (mInProgress)
    {
      mCommand.clear();
      mInProgress = false;
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Command canceled.")));
    }

    mPromise.reset(new std::promise<int>());

    mCommand = command;
    mInProgress = true;
    mExecutionStartTime = std::chrono::system_clock::now();
    mStartPosition.clear();
    mTimeout = timeout;
    for (size_t i = 0; i < mSkeleton->getDofs().size(); i++)
    {
      auto dof = mSkeleton->getDof(i);
      mStartPosition.push_back(dof->getPosition());
      if (mTimeout.count() > 1E-8)
      {
        dof->setVelocity((command[i] - mStartPosition[i]) / mTimeout.count());
      }
    }
  }

  return mPromise->get_future();
}

//==============================================================================
void KinematicSimulationPositionExecutor::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInProgress)
  {
    return;
  }

  const auto timeSinceBeginning = timepoint - mExecutionStartTime;
  const auto executionTime
      = std::chrono::duration<double>(timeSinceBeginning).count();

  // Instantaneous movement if no timeout.
  double interpTime
      = (mTimeout.count() == 0) ? 1.0 : executionTime / mTimeout.count();
  // Stop at the correct position
  if (interpTime > 1.0)
  {
    interpTime = 1.0;
  }

  // executionTime may be negative if the thread calling \c step is queued
  // before and dequeued after \c execute is called.
  if (executionTime < 0)
  {
    return;
  }

  for (size_t i = 0; i < mSkeleton->getDofs().size(); i++)
  {
    auto dof = mSkeleton->getDof(i);
    double pose
        = (1.0 - interpTime) * mStartPosition[i] + interpTime * mCommand[i];
    dof->setPosition(pose);
  }

  // Check if command has timed out
  if (executionTime >= mTimeout.count())
  {
    mCommand.clear();
    mInProgress = false;
    mPromise->set_value(0);
  }
}

//==============================================================================
void KinematicSimulationPositionExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress)
  {
    mCommand.clear();
    mInProgress = false;
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Command canceled.")));
  }
  else
  {
    dtwarn << "[KinematicSimulationPositionExecutor::cancel] Attempting to "
           << "cancel command, but no command in progress.\n";
  }
}

} // namespace control
} // namespace aikido
