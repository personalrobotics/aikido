#ifndef AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_

#include <chrono>
#include <future>
#include <set>
#include "aikido/common/pointers.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(TrajectoryExecutor)

/// Abstract class for executing trajectories.
class TrajectoryExecutor
{
public:
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

  /// Abort the current trajectory.
  /// \note This is currently only supported in simulation.
  virtual void abort() = 0;

protected:
  /// Set of trajectories validated by executor
  std::set<const trajectory::Trajectory*> mValidatedTrajectories;

  /// Time of previous call
  std::chrono::system_clock::time_point mExecutionStartTime;
};

} // namespace control
} // namespace aikido

#endif
