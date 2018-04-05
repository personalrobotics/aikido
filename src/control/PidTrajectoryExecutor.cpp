#include "aikido/control/PidTrajectoryExecutor.hpp"
#include <dart/common/StlHelpers.hpp>
#include "aikido/common/algorithm.hpp"
#include "aikido/control/TrajectoryRunningException.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace control {

//==============================================================================
PidTrajectoryExecutor::PidTrajectoryExecutor(
    ::dart::dynamics::SkeletonPtr skeleton,
    ::dart::simulation::WorldPtr world,
    aikido::control::PidGains gains)
  : mSkeleton{std::move(skeleton)}
  , mWorld{std::move(world)}
  , mGains{std::move(gains)}
  , mTraj{nullptr}
  , mStateSpace{nullptr}
  , mInProgress{false}
  , mPromise{nullptr}
  , mMutex{}
{
  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");

  bool skeletonInWorld = false;
  for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i)
  {
    if (mWorld->getSkeleton(i) == mSkeleton)
      skeletonInWorld = true;
  }
  if (!skeletonInWorld)
    throw std::invalid_argument("Skeleton is not in World.");

  mProportionalError = Eigen::VectorXd::Zero(mSkeleton->getNumDofs());
  mIntegralError = Eigen::VectorXd::Zero(mSkeleton->getNumDofs());
  mDerivativeError = Eigen::VectorXd::Zero(mSkeleton->getNumDofs());
}

//==============================================================================
PidTrajectoryExecutor::~PidTrajectoryExecutor()
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
void PidTrajectoryExecutor::validate(const trajectory::Trajectory* traj)
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

  // TODO: Delete this line once the skeleton is locked by isCompatible
  std::lock_guard<std::mutex> lock(mSkeleton->getMutex());
  space->checkCompatibility(mSkeleton.get());

  mValidatedTrajectories.emplace(traj);
}

//==============================================================================
std::future<void> PidTrajectoryExecutor::execute(
    const trajectory::ConstTrajectoryPtr& traj)
{
  validate(traj.get());

  {
    std::lock_guard<std::mutex> lock(mMutex);
    DART_UNUSED(lock); // Suppress unused variable warning

    if (mInProgress)
      throw TrajectoryRunningException();

    mPromise.reset(new std::promise<void>());

    mTraj = traj;
    mStateSpace = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(
        traj->getStateSpace());
    mInProgress = true;
    mExecutionStartTime = std::chrono::system_clock::now();
    mTimeOfPreviousCall = mExecutionStartTime;
  }

  return mPromise->get_future();
}

//==============================================================================
void PidTrajectoryExecutor::step(
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
  }
  else if (mInProgress && !mTraj)
  {
    mPromise->set_exception(
        std::make_exception_ptr(
            std::runtime_error(
                "Set for execution but no trajectory is provided.")));
    mInProgress = false;
  }

  const auto timeSinceBeginning = timepoint - mExecutionStartTime;
  const auto executionTime
      = std::chrono::duration<double>(timeSinceBeginning).count();

  const auto timeSincePreviousCall = timepoint - mTimeOfPreviousCall;
  const auto dt = std::chrono::duration<double>(timeSincePreviousCall).count();

  if (executionTime < 0)
    throw std::invalid_argument("Timepoint is before execution start time.");
  if (dt < 0)
    throw std::invalid_argument("Timepoint is before previous call.");

  mTimeOfPreviousCall = timepoint;

  Eigen::VectorXd desiredPositions, desiredVelocities;
  auto state = mStateSpace->createState();
  mTraj->evaluate(executionTime, state);
  mStateSpace->convertStateToPositions(state, desiredPositions);
  mTraj->evaluateDerivative(executionTime, 1, desiredVelocities);

  // TODO: does it make sense to support PID control of trajectories that
  // specify subsets of mSkeleton's DOFs?
  Eigen::VectorXd actualPositions = mSkeleton->getPositions();
  Eigen::VectorXd actualVelocities = mSkeleton->getVelocities();

  mProportionalError = desiredPositions - actualPositions;
  mIntegralError += dt * mProportionalError;
  mDerivativeError = desiredVelocities - actualVelocities;

  for (std::size_t i = 0; i < static_cast<std::size_t>(mIntegralError.size());
       ++i)
    mIntegralError[i] = aikido::common::clamp(
        mIntegralError[i], mGains.mIntegralMin[i], mGains.mIntegralMax[i]);

  Eigen::VectorXd efforts
      = (mGains.mProportionalGains.array() * mProportionalError.array())
        + (mGains.mIntegralGains.array() * mIntegralError.array())
        + (mGains.mDerivativeGains.array() * mDerivativeError.array());

  // Simulate in World
  mWorld->setTimeStep(dt);
  mSkeleton->setCommands(efforts);
  mWorld->step();

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
void PidTrajectoryExecutor::abort()
{
  std::lock_guard<std::mutex> lock(mMutex);

  if (mInProgress && mTraj)
  {
    mTraj.reset();
    mStateSpace.reset();
    mInProgress = false;
    mPromise->set_exception(
        std::make_exception_ptr(std::runtime_error("Trajectory aborted.")));
  }
}

} // namespace control
} // namespace aikido
