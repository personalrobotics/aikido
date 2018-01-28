#ifndef AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_

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
  TrajectoryExecutor(double timestep);

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
  virtual std::future<void> execute(trajectory::TrajectoryPtr _traj) = 0;

  /// Step once.
  virtual void step() = 0;

  /// Abort the current trajectory.
  /// \note This is currently only supported in simulation.
  virtual void abort() = 0;

protected:
  /// Time period that each call to step() should simulate
  double mTimestep;
};

using TrajectoryExecutorPtr = std::shared_ptr<TrajectoryExecutor>;

} // namespace control
} // namespace aikido

#endif
