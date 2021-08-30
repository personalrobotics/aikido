#include "aikido/control/KinematicSimulationPositionExecutor.hpp"

#include <dart/common/Console.hpp>

#include "aikido/common/memory.hpp"
#include "aikido/control/TrajectoryRunningException.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;

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
{
  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");
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
          std::make_exception_ptr(std::runtime_error("Trajectory canceled.")));
    }
  }
}

//==============================================================================
std::future<int> KinematicSimulationPositionExecutor::execute(
    const std::vector<double> command,
    const std::chrono::duration<double>& timeout)
{
  using aikido::statespace::dart::MetaSkeletonStateSpacePtr;

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning
    auto promise = new std::promise<int>();

    if (mInProgress)
    {
      promise->set_exception(std::make_exception_ptr(
          std::runtime_error("Another position command is in progress.")));
      return promise->get_future();
    }

    if (command.size() != mJoints.size())
    {
      promise->set_exception(std::make_exception_ptr(
          std::runtime_error("DOF of command does not match DOF of joints.")));
      return promise->get_future();
    }

    mPromise.reset(promise);

    mCommand = command;
    mInProgress = true;
    mExecutionStartTime = std::chrono::system_clock::now();
    mStartPosition.clear();
    for (size_t i; i < mSkeleton->getDofs().size(); i++)
    {
      auto dof = mSkeleton->getDof(i);
      if (mTimeout.count() > 1E-8)
      {
        dof->setVelocity((command[i] - mStartPosition[i]) / mTimeout.count());
      }
      mStartPosition.push_back(dof->getPosition());
    }
    mTimeout = timeout;
  }

  return mPromise->get_future();
}

//==============================================================================
void KinematicSimulationPositionExecutor::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInProgress)
    return;

  const auto timeSinceBeginning = timepoint - mExecutionStartTime;
  const auto executionTime
      = std::chrono::duration<double>(timeSinceBeginning).count();

  // Instantaneous movement if no timeout.
  double interpTime
      = (mTimeout.count() == 0) ? 1.0 : executionTime / mTimeout.count();

  // executionTime may be negative if the thread calling \c step is queued
  // before and dequeued after \c execute is called.
  if (executionTime < 0)
    return;

  for (size_t i; i < mSkeleton->getDofs().size(); i++)
  {
    auto dof = mSkeleton->getDof(i);
    dof->setPosition(
        (1.0 - interpTime) * mStartPosition[i] + interpTime * mCommand[i]);
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
        std::make_exception_ptr(std::runtime_error("Trajectory canceled.")));
  }
  else
  {
    dtwarn << "[KinematicSimulationPositionExecutor::cancel] Attempting to "
           << "cancel command, but no command in progress.\n";
  }
}

} // namespace control
} // namespace aikido
