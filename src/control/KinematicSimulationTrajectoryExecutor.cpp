#include "aikido/control/KinematicSimulationTrajectoryExecutor.hpp"

#include <dart/common/Console.hpp>

#include "aikido/common/memory.hpp"
#include "aikido/control/TrajectoryRunningException.hpp"
#include "aikido/control/util.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace control {

//==============================================================================
KinematicSimulationTrajectoryExecutor::KinematicSimulationTrajectoryExecutor(
    ::dart::dynamics::MetaSkeletonPtr metaskeleton)
// Edits MetaSkeleton DoFs directly
  : TrajectoryExecutor(checkNull(metaskeleton)->getDofs(), std::set<ExecutorType>{ExecutorType::STATE})
  , mMetaSkeleton{metaskeleton}
  , mTraj{nullptr}
  , mStateSpace{nullptr}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  // Skeleton checked by checkNull already
  stop();
}

//==============================================================================
KinematicSimulationTrajectoryExecutor::~KinematicSimulationTrajectoryExecutor()
{
  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress && mTraj)
    {
      mInProgress = false;
      mPromise->set_exception(
          std::make_exception_ptr(std::runtime_error("Trajectory canceled.")));
    }
    stop();
  }
}

//==============================================================================
void KinematicSimulationTrajectoryExecutor::validate(
    const trajectory::Trajectory* traj)
{
  if (!traj)
    throw std::invalid_argument("Trajectory is null.");

  if (mValidatedTrajectories.find(traj) != mValidatedTrajectories.end())
    return;

  const auto space = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
      traj->getStateSpace());

  if (!space)
  {
    throw std::invalid_argument(
        "Trajectory is not in a MetaSkeletonStateSpace.");
  }

  // Check that traj space is compatible with metaskeleton
  if (!space->isCompatible(mMetaSkeleton.get())) {
    throw std::invalid_argument(
      "Trajectory StateSpace incompatible with MetaSkeleton");
  }

  mValidatedTrajectories.emplace(traj);
}

//==============================================================================
std::future<void> KinematicSimulationTrajectoryExecutor::execute(
    const trajectory::ConstTrajectoryPtr& traj)
{
  using aikido::statespace::dart::MetaSkeletonStateSpacePtr;

  validate(traj.get());

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
      throw TrajectoryRunningException();

    mPromise.reset(new std::promise<void>());

    mTraj = std::move(traj);
    mStateSpace = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
        mTraj->getStateSpace());
    mInProgress = true;
    mExecutionStartTime = std::chrono::system_clock::now();
  }

  return mPromise->get_future();
}

//==============================================================================
void KinematicSimulationTrajectoryExecutor::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInProgress && !mTraj)
    return;

  if (!mInProgress && mTraj)
  {
    mPromise->set_exception(std::make_exception_ptr(
        std::runtime_error("Trajectory terminated while in execution.")));
    mTraj.reset();
  }
  else if (mInProgress && !mTraj)
  {
    mPromise->set_exception(std::make_exception_ptr(std::runtime_error(
        "Set for execution but no trajectory is provided.")));
    mInProgress = false;
  }

  const auto timeSinceBeginning = timepoint - mExecutionStartTime;
  const auto executionTime
      = std::chrono::duration<double>(timeSinceBeginning).count();

  // executionTime may be negative if the thread calling \c step is queued
  // before and dequeued after \c execute is called.
  if (executionTime < 0)
    return;

  auto state = mStateSpace->createState();
  mTraj->evaluate(executionTime, state);

  mStateSpace->setState(mMetaSkeleton.get(), state);

  // Check if trajectory has completed.
  if (executionTime >= mTraj->getEndTime())
  {
    mTraj.reset();
    mStateSpace.reset();
    mInProgress = false;
    mPromise->set_value();
  }
}

//==============================================================================
void KinematicSimulationTrajectoryExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress && mTraj)
  {
    mTraj.reset();
    mStateSpace.reset();
    mInProgress = false;
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Trajectory canceled.")));
  }
  else
  {
    dtwarn << "[KinematicSimulationTrajectoryExecutor::cancel] Attempting to "
           << "cancel trajectory, but no trajectory in progress.\n";
  }
}

} // namespace control
} // namespace aikido
