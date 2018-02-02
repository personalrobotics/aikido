#include "aikido/control/KinematicSimulationTrajectoryExecutor.hpp"
#include <dart/common/StlHelpers.hpp>
#include "aikido/control/TrajectoryRunningException.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace control {

//==============================================================================
KinematicSimulationTrajectoryExecutor::KinematicSimulationTrajectoryExecutor(
    ::dart::dynamics::SkeletonPtr skeleton, std::chrono::milliseconds timestep)
  : TrajectoryExecutor(timestep)
  , mSkeleton{std::move(skeleton)}
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
    trajectory::TrajectoryPtr traj)
{
  if (!traj)
    throw std::invalid_argument("Traj is null.");

  if (traj->metadata.executorValidated)
    return;

  const auto space = std::dynamic_pointer_cast<MetaSkeletonStateSpace>(
      traj->getStateSpace());

  if (!space)
    throw std::invalid_argument(
        "Trajectory is not in a MetaSkeletonStateSpace.");

  // TODO: Delete this line once the skeleton is locked by isCompatible
  std::lock_guard<std::mutex> lock(mSkeleton->getMutex());
  space->checkCompatibility(mSkeleton.get());

  traj->metadata.executorValidated = true;
}

//==============================================================================
std::future<void> KinematicSimulationTrajectoryExecutor::execute(
    trajectory::TrajectoryPtr traj)
{
  validate(traj);

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
      throw TrajectoryRunningException();

    mPromise.reset(new std::promise<void>());

    mTraj = traj;
    mStateSpace = std::dynamic_pointer_cast<MetaSkeletonStateSpace>(
        traj->getStateSpace());
    mInProgress = true;
    mExecutionTime = 0;
  }

  return mPromise->get_future();
}

//==============================================================================
void KinematicSimulationTrajectoryExecutor::step()
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
  }
  else if (mInProgress && !mTraj)
  {
    mPromise->set_exception(
        std::make_exception_ptr(
            std::runtime_error(
                "Set for execution but no trajectory is provided.")));
    mInProgress = false;
  }

  const auto period = std::chrono::duration<double>(mTimestep).count();
  mExecutionTime += period;

  auto state = mStateSpace->createState();
  mTraj->evaluate(mExecutionTime, state);
  mStateSpace->setState(mSkeleton.get(), state);

  // Check if trajectory has completed.
  if (mExecutionTime >= mTraj->getEndTime())
  {
    mTraj.reset();
    mStateSpace.reset();
    mInProgress = false;
    mExecutionTime = 0;
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
    mInProgress = false;
    mExecutionTime = 0;
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Trajectory aborted.")));
  }
}

} // namespace control
} // namespace aikido
