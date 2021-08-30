#include "aikido/control/InstantaneousTrajectoryExecutor.hpp"

#include "aikido/control/TrajectoryRunningException.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace control {

//==============================================================================
InstantaneousTrajectoryExecutor::InstantaneousTrajectoryExecutor(
    ::dart::dynamics::SkeletonPtr skeleton)
  : TrajectoryExecutor(skeletonToJointNames(skeleton))
  , mSkeleton{std::move(skeleton)}
  , mPromise{nullptr}
  , mMutex{}
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
  if (!traj)
    throw std::invalid_argument("Trajectory is null.");

  if (mValidatedTrajectories.find(traj) != mValidatedTrajectories.end())
    return;

  const auto space = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
      traj->getStateSpace());

  if (!space)
    throw std::invalid_argument(
        "Trajectory is not in a MetaSkeletonStateSpace.");

  // TODO: Delete this line once the skeleton is locked by isCompatible
  std::lock_guard<std::mutex> lock(mSkeleton->getMutex());
  space->checkIfContained(mSkeleton.get());

  mValidatedTrajectories.emplace(traj);
}

//==============================================================================
std::future<void> InstantaneousTrajectoryExecutor::execute(
    const trajectory::ConstTrajectoryPtr& traj)
{
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
void InstantaneousTrajectoryExecutor::cancel()
{
  // Do nothing
}

} // namespace control
} // namespace aikido
