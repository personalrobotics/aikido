#include "aikido/control/KinematicSimulationTrajectoryExecutor.hpp"
#include <dart/common/StlHelpers.hpp>
#include "aikido/control/TrajectoryRunningException.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace control {

//==============================================================================
KinematicSimulationTrajectoryExecutor::KinematicSimulationTrajectoryExecutor(
    ::dart::dynamics::SkeletonPtr skeleton)
  : mSkeleton{std::move(skeleton)}
  , mTraj{nullptr}
  , mStateSpace{nullptr}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");
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
          std::make_exception_ptr(std::runtime_error("Trajectory aborted.")));
    }
  }
}

//==============================================================================
void KinematicSimulationTrajectoryExecutor::validate(
    const trajectory::Trajectory* traj)
{
  // TODO (avk): Same as in InstantaneousTrajectoryExecutor

  if (!traj)
    throw std::invalid_argument("Traj is null.");

  if (traj->metadata.executorValidated)
    return;

  const auto space = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
      traj->getStateSpace());

  if (!space)
    throw std::invalid_argument(
        "Trajectory is not in a MetaSkeletonStateSpace.");

  // TODO: Delete this line once the skeleton is locked by isCompatible
  std::lock_guard<std::mutex> lock(mSkeleton->getMutex());

  space->checkIfContained(mSkeleton.get());

  traj->metadata.executorValidated = true;
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
    mMetaSkeleton = mStateSpace->getControlledMetaSkeleton(mSkeleton);

    if (!mMetaSkeleton)
      throw std::invalid_argument("Failed to create MetaSkeleton");
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
    mPromise->set_exception(
        std::make_exception_ptr(
            std::runtime_error("Trajectory terminated while in execution.")));
    mTraj.reset();
    mMetaSkeleton.reset();
  }
  else if (mInProgress && !mTraj)
  {
    mPromise->set_exception(
        std::make_exception_ptr(
            std::runtime_error(
                "Set for execution but no trajectory is provided.")));
    mMetaSkeleton.reset();
    mInProgress = false;
  }

  const auto timeSinceBeginning = timepoint - mExecutionStartTime;
  const auto executionTime
      = std::chrono::duration<double>(timeSinceBeginning).count();

  if (executionTime < 0)
    throw std::invalid_argument("Timepoint is before execution start time.");

  auto state = mStateSpace->createState();
  mTraj->evaluate(executionTime, state);

  mStateSpace->setState(mMetaSkeleton.get(), state);

  // Check if trajectory has completed.
  if (executionTime >= mTraj->getEndTime())
  {
    mTraj.reset();
    mStateSpace.reset();
    mMetaSkeleton.reset();
    mInProgress = false;
    mPromise->set_value();
  }
}

//==============================================================================
void KinematicSimulationTrajectoryExecutor::abort()
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress && mTraj)
  {
    mTraj.reset();
    mStateSpace.reset();
    mMetaSkeleton.reset();
    mInProgress = false;
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Trajectory aborted.")));
  }
}

} // namespace control
} // namespace aikido
