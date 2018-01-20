#ifndef AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_

#include <future>
#include "aikido/trajectory/smart_pointer.hpp"

namespace aikido {
namespace control {

class TrajectoryExecutor
{
public:
  virtual ~TrajectoryExecutor() = default;

  /// Validate the traj in preparation for execution.
  /// \param traj Trajectory to be validated
  virtual void validate(trajectory::TrajectoryPtr traj) = 0;

  /// Validate and execute traj, setting future upon completion. If a trajectory
  /// is already running, raise an exception unless the executor supports
  /// queuing.
  /// \param traj Trajectory to be executed.
  virtual std::future<void> execute(trajectory::TrajectoryPtr _traj) = 0;

  /// Executes one step.
  virtual void step() = 0;

  /// Aborts the current trajectory. This is currently only supported in
  /// simulation.
  virtual void abort() = 0;
};

} // namespace control
} // namespace aikido

#endif
