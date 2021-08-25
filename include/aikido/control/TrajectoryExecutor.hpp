#ifndef AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>

#include "aikido/common/pointers.hpp"
#include "aikido/trajectory/Trajectory.hpp"
#include "aikido/control/Executor.hpp"

#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>

namespace aikido {
namespace control {

// inline function to get joint names from skeleton
inline std::vector<std::string> skeletonToJointNames(dart::dynamics::SkeletonPtr skeleton) {
  std::vector<std::string> ret;
  ret.reserve(skeleton->getNumDofs());
  for (auto dof : skeleton->getDofs()) {
    ret.push_back(dof->getName());
  }
  return ret;
}

AIKIDO_DECLARE_POINTERS(TrajectoryExecutor)

/// Abstract class for executing trajectories.
class TrajectoryExecutor : public Executor
{
public:
  TrajectoryExecutor(std::vector<std::string> joints) : Executor(ExecutorType::kTRAJECTORY, joints) {}
  virtual ~TrajectoryExecutor() = default;

  /// Validate the traj in preparation for execution.
  ///
  /// \param traj Trajectory to be validated
  virtual void validate(const trajectory::Trajectory* traj) = 0;

  /// Validate and execute traj, setting future upon completion. If a trajectory
  /// is already running, raise an exception unless the executor supports
  /// queuing.
  ///
  /// \param traj Trajectory to be executed.
  /// \return future<void> for trajectory execution. If trajectory terminates
  ///        before completion, future will be set to a runtime_error.
  virtual std::future<void> execute(const trajectory::ConstTrajectoryPtr& traj)
      = 0;

  /// Step to a point in time.
  /// \note \c timepoint can be a time in the future to enable faster than
  /// real-time execution.
  ///
  /// \param timepoint Time to simulate to
  virtual void step(const std::chrono::system_clock::time_point& timepoint) = 0;

  /// Cancel the current trajectory.
  virtual void cancel() = 0;

protected:
  /// Set of trajectories validated by executor
  std::set<const trajectory::Trajectory*> mValidatedTrajectories;

  /// Time of previous call
  std::chrono::system_clock::time_point mExecutionStartTime;
};

} // namespace control
} // namespace aikido

#endif
