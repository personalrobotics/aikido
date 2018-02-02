#ifndef AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_

#include <chrono>
#include <future>
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace control {

/// Abstract class for executing trajectories.
class TrajectoryExecutor
{
public:
  /// Constructor.
  ///
  /// \param timestep The time period that each call to step() should simulate
  TrajectoryExecutor(std::chrono::milliseconds timestep);

  virtual ~TrajectoryExecutor() = default;

  /// Validate the traj in preparation for execution.
  ///
  /// \param traj Trajectory to be validated
  virtual void validate(trajectory::TrajectoryPtr traj) = 0;

  /// Validate and execute traj, setting future upon completion. If a trajectory
  /// is already running, raise an exception unless the executor supports
  /// queuing.
  ///
  /// \param traj Trajectory to be executed.
  /// \return future<void> for trajectory execution. If trajectory terminates
  ///        before completion, future will be set to a runtime_error.
  virtual std::future<void> execute(trajectory::TrajectoryPtr _traj) = 0;

  /// Step once.
  virtual void step() = 0;

  /// Abort the current trajectory.
  /// \note This is currently only supported in simulation.
  virtual void abort() = 0;

  /// Get the current timestep.
  virtual std::chrono::milliseconds getTimestep() const;

  /// Set the current timestep.
  virtual void setTimestep(std::chrono::milliseconds timestep);

protected:
  /// Time period that each call to step() should simulate
  std::chrono::milliseconds mTimestep;
};

using TrajectoryExecutorPtr = std::shared_ptr<TrajectoryExecutor>;

} // namespace control
} // namespace aikido

#endif
