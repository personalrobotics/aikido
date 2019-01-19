#include "aikido/control/KinematicSimulationTrajectoryExecutor.hpp"
#include <dart/common/Console.hpp>
#include <dart/common/StlHelpers.hpp>
#include "aikido/control/TrajectoryRunningException.hpp"
#include "aikido/statespace/Rn.hpp"
#include "aikido/statespace/dart/CartesianProductMetaSkeletonStateSpace.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::CartesianProductMetaSkeletonStateSpace;
using aikido::statespace::R1;

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
          std::make_exception_ptr(std::runtime_error("Trajectory canceled.")));
    }
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

  auto space = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
      traj->getStateSpace());

  if (!space)
  {
    auto cpSpace = std::dynamic_pointer_cast<const CartesianProductMetaSkeletonStateSpace>(
      traj->getStateSpace());

    if (cpSpace)
    {
      // Check that all joints are R1 state spaces.
      for (std::size_t i = 0; i < cpSpace->getNumSubspaces(); ++i)
      {
        auto subSpace = cpSpace->getSubspace(i);
        auto r1 = std::dynamic_pointer_cast<const R1>(subSpace);
        if (!r1)
        {
          std::stringstream message;
          message << "Trajectory is not in a MetaSkeletonStateSpace"
          << " or in CartesianProductMetaSkeletonStateSpace of R1 spaces.";
          throw std::invalid_argument(message.str());
        }
      }
    }
    else
    {
      std::stringstream message;
      message << "Trajectory is not in a MetaSkeletonStateSpace"
      << "nor in CartesianProductMetaSkeletonStateSpace.";
      throw std::invalid_argument(message.str());
    }
  }
  else
  {
    // TODO: Delete this line once the skeleton is locked by isCompatible
    std::lock_guard<std::mutex> lock(mSkeleton->getMutex());
    space->checkIfContained(mSkeleton.get());
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

    if (!mStateSpace)
    {
      auto cpSpace = std::dynamic_pointer_cast<const CartesianProductMetaSkeletonStateSpace>(
        mTraj->getStateSpace());
      if (!cpSpace)
        throw std::invalid_argument("Invalid");
      mStateSpace = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
        cpSpace->getMetaSkeletonStateSpace());
    }

    if (!mStateSpace)
      throw std::invalid_argument(
        "Statespace needs to be either MetaSkeletonStateSpace or CartesianProduct.");

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

  // executionTime may be negative if the thread calling \c step is queued
  // before and dequeued after \c execute is called.
  if (executionTime < 0)
    return;

  auto cpSpace = std::dynamic_pointer_cast<const CartesianProductMetaSkeletonStateSpace>(
        mTraj->getStateSpace());

  if (!cpSpace)
  {
    auto state = mStateSpace->createState();
    mTraj->evaluate(executionTime, state);
    mStateSpace->setState(mMetaSkeleton.get(), state);
  }
  else
  {
    auto state = cpSpace->createState();
    mTraj->evaluate(executionTime, state);

    // Convert to position vector and then set the metaskeleton.
    Eigen::VectorXd tangent;
    cpSpace->logMap(state, tangent);
    mMetaSkeleton->setPositions(tangent);
  }

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
void KinematicSimulationTrajectoryExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress && mTraj)
  {
    mTraj.reset();
    mStateSpace.reset();
    mMetaSkeleton.reset();
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
