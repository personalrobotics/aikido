#include <chrono>
#include <thread>
#include <dart/common/StlHelpers.hpp>
#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/statespace/SO2.hpp>

using aikido::statespace::SO2;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;

namespace aikido {
namespace control {

//==============================================================================
KinematicSimulationTrajectoryExecutor::KinematicSimulationTrajectoryExecutor(
    ::dart::dynamics::SkeletonPtr skeleton)
  : TrajectoryExecutor()
  , mSkeleton(std::move(skeleton))
  , mPromise(nullptr)
  , mTraj(nullptr)
  , mMutex()
  , mInExecution(false)
{
  if (!mSkeleton)
    throw std::invalid_argument("Skeleton is null.");
}

//==============================================================================
KinematicSimulationTrajectoryExecutor::~KinematicSimulationTrajectoryExecutor()
{
  {
    std::lock_guard<std::mutex> lockSpin(mMutex);
    mInExecution = false;
  }
}

//==============================================================================
std::future<void> KinematicSimulationTrajectoryExecutor::execute(
    trajectory::TrajectoryPtr traj, bool skip)
{
  if (!traj)
    throw std::invalid_argument("Traj is null.");

  auto space = std::dynamic_pointer_cast<MetaSkeletonStateSpace>(
      traj->getStateSpace());

  if (!space)
    throw std::invalid_argument(
        "Trajectory does not operate in this Executor's"
        " MetaSkeletonStateSpace.");

  auto metaSkeleton = space->getMetaSkeleton();

  // Check if metaSkeleton contains Dofs only in mSkeleton.
  std::unique_lock<std::mutex> skeleton_lock(mSkeleton->getMutex());
  for (auto dof : metaSkeleton->getDofs())
  {
    auto name = dof->getName();
    auto dof_in_skeleton = mSkeleton->getDof(name);

    if (!dof_in_skeleton)
    {
      std::stringstream msg;
      msg << "traj contrains dof [" << name << "], which is not in mSkeleton.";

      throw std::invalid_argument(msg.str());
    }
  }
  skeleton_lock.unlock();

  {
    std::lock_guard<std::mutex> lockSpin(mMutex);

    if (mTraj)
      throw std::runtime_error("Another trajectory in execution.");

    mPromise.reset(new std::promise<void>());
    if (skip)
    {
      auto state = space->createState();
      traj->evaluate(traj->getEndTime(), state);

      Eigen::VectorXd positions(space->getDimension());
      space->convertStateToPositions(state, positions);
      metaSkeleton->setPositions(positions);
      mPromise->set_value();
    }
    else
    {
      mTraj = traj;
      mInExecution = true;
      mExecutionStartTime = std::chrono::system_clock::now();
    }
  }

  return mPromise->get_future();
}

//==============================================================================
void KinematicSimulationTrajectoryExecutor::step()
{
  using namespace std::chrono;
  std::lock_guard<std::mutex> lock(mMutex);

  if (!mInExecution && !mTraj)
    return;

  if (!mInExecution && mTraj)
  {
    mPromise->set_exception(
        std::make_exception_ptr(
            std::runtime_error("Trajectory terminated while in execution.")));
    mTraj.reset();
  }
  else if (mInExecution && !mTraj)
  {
    mPromise->set_exception(
        std::make_exception_ptr(
            std::runtime_error(
                "Set for execution but no trajectory is provided.")));
    mInExecution = false;
  }

  auto timeSinceBeginning = system_clock::now() - mExecutionStartTime;
  auto tsec = duration_cast<std::chrono::duration<double>>(timeSinceBeginning)
                  .count();

  // Can't do static here because MetaSkeletonStateSpace inherits
  // CartesianProduct which inherits virtual StateSpace
  auto space = std::dynamic_pointer_cast<MetaSkeletonStateSpace>(
      mTraj->getStateSpace());
  auto metaSkeleton = space->getMetaSkeleton();
  auto state = space->createState();

  mTraj->evaluate(tsec, state);

  space->setState(state);

  // Check if trajectory has completed.
  bool const is_done = (tsec >= mTraj->getEndTime());
  if (is_done)
  {
    mTraj.reset();
    mPromise->set_value();
    mInExecution = false;
  }
}

} // namespace control
} // namespace aikido
