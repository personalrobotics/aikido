#include "aikido/control/KinematicSimulationVelocityExecutor.hpp"

#include <dart/common/Console.hpp>

#include "aikido/common/memory.hpp"

namespace aikido {
namespace control {

//==============================================================================
KinematicSimulationVelocityExecutor::KinematicSimulationVelocityExecutor(
    ::dart::dynamics::SkeletonPtr skeleton)
  : VelocityExecutor(skeletonToJointNames(skeleton))
  , mSkeleton{std::move(skeleton)}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");
}

//==============================================================================
KinematicSimulationVelocityExecutor::~KinematicSimulationVelocityExecutor()
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
std::future<int> KinematicSimulationVelocityExecutor::execute(
    const std::vector<double> command,
    const std::chrono::duration<double>& timeout)
{

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning
    auto promise = std::promise<int>();

    if (mInProgress)
    {
      promise.set_exception(std::make_exception_ptr(
          std::runtime_error("Another position command is in progress.")));
      return promise.get_future();
    }

    if (command.size() != mJoints.size())
    {
      promise.set_exception(std::make_exception_ptr(
          std::runtime_error("DOF of command does not match DOF of joints.")));
      return promise.get_future();
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
      dof->setVelocity(command[i]);
      mStartPosition.push_back(dof->getPosition());
    }
  }

  return mPromise->get_future();
}

//==============================================================================
void KinematicSimulationVelocityExecutor::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInProgress)
    return;

  const auto timeSinceBeginning = timepoint - mExecutionStartTime;
  const auto executionTime
      = std::chrono::duration<double>(timeSinceBeginning).count();

  // executionTime may be negative if the thread calling \c step is queued
  // before and dequeued after \c execute is called.
  if (executionTime < 0)
    return;

  for (size_t i = 0; i < mSkeleton->getDofs().size(); i++)
  {
    auto dof = mSkeleton->getDof(i);
    dof->setPosition(mStartPosition[i] + executionTime * mCommand[i]);
  }

  // Check if command has timed out
  if (mTimeout.count() > 0.0 && executionTime >= mTimeout.count())
  {
    mCommand.clear();
    mInProgress = false;
    mPromise->set_value(0);
  }
}

//==============================================================================
void KinematicSimulationVelocityExecutor::cancel()
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
    dtwarn << "[KinematicSimulationVelocityExecutor::cancel] Attempting to "
           << "cancel command, but no command in progress.\n";
  }
}

} // namespace control
} // namespace aikido
