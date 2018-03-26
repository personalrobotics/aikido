#include "aikido/control/InstantaneousTrajectoryExecutor.hpp"
#include "aikido/control/TrajectoryRunningException.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace control {

//==============================================================================
InstantaneousTrajectoryExecutor::InstantaneousTrajectoryExecutor(
    ::dart::dynamics::SkeletonPtr skeleton)
  : mSkeleton{std::move(skeleton)}, mPromise{nullptr}, mMutex{}
{
  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");
}

//==============================================================================
InstantaneousTrajectoryExecutor::~InstantaneousTrajectoryExecutor()
{
  // Do nothing.
}

//==============================================================================
void InstantaneousTrajectoryExecutor::validate(
    const trajectory::Trajectory* traj)
{
  /*
   * Introduce a data structure to store the trajectories that have been
   * validated by the executor. In this function, check for the existence
   * of the trajectory in the data structure. If it is present,
   * consider it to already have been validated. Otherwise validate it.
   * This is because the compatibility is being checked against the
   * executor's mSkeleton's space against trajectory's space, and the
   * executor validates.
  */

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
  space->checkCompatibility(mSkeleton.get());

//  traj->metadata.executorValidated = true;
}

//==============================================================================
std::future<void> InstantaneousTrajectoryExecutor::execute(
    const trajectory::ConstTrajectoryPtr& traj)
{
  // So validate traj can change member of executor, not traj
  validate(traj.get());

  const auto space = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
      traj->getStateSpace());

  {
    std::lock_guard<std::mutex> lock(mMutex);

    mPromise.reset(new std::promise<void>());

    auto state = space->createState();
    traj->evaluate(traj->getEndTime(), state);
    space->setState(mSkeleton.get(), state);
    mPromise->set_value();
  }

  return mPromise->get_future();
}

//==============================================================================
void InstantaneousTrajectoryExecutor::step(
    const std::chrono::system_clock::time_point& /*timepoint*/)
{
  // Do nothing
}

//==============================================================================
void InstantaneousTrajectoryExecutor::abort()
{
  // Do nothing
}

} // namespace control
} // namespace aikido
