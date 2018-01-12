#include <aikido/control/InstantaneousTrajectoryExecutor.hpp>
#include <aikido/control/TrajectoryRunningException.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

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
void InstantaneousTrajectoryExecutor::validate(trajectory::TrajectoryPtr traj)
{
  if (!traj)
    throw std::invalid_argument("Traj is null.");

  if (traj->metadata.executorValidated)
    return;

  const auto space = std::dynamic_pointer_cast<MetaSkeletonStateSpace>(
      traj->getStateSpace());

  if (!space)
    throw std::invalid_argument(
        "Trajectory does not operate in this Executor's"
        " MetaSkeletonStateSpace.");

  // Check if the space only contains DOFs in mSkeleton.
  std::unique_lock<std::mutex> skeleton_lock(mSkeleton->getMutex());
  for (const auto& name : space->getProperties().getDofNames())
  {
    auto dof_in_skeleton = mSkeleton->getDof(name);

    if (!dof_in_skeleton)
    {
      std::stringstream msg;
      msg << "traj contains dof [" << name << "], which is not in mSkeleton.";

      throw std::invalid_argument(msg.str());
    }
  }
  skeleton_lock.unlock();

  traj->metadata.executorValidated = true;
}
//==============================================================================
std::future<void> InstantaneousTrajectoryExecutor::execute(
    trajectory::TrajectoryPtr traj)
{
  validate(traj);

  const auto space = std::dynamic_pointer_cast<MetaSkeletonStateSpace>(
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
void InstantaneousTrajectoryExecutor::step()
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
