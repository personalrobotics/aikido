#ifndef AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_

#include <future>
#include <aikido/trajectory/Trajectory.hpp>
#include "TrajectoryResult.hpp"

namespace aikido {
namespace control {

class TrajectoryExecutor
{
public:
  virtual ~TrajectoryExecutor() = default;

  /// Execute traj and set future upon completion.
  /// \param traj Trajectory to be executed.
  virtual std::future<void> execute(trajectory::TrajectoryPtr _traj) = 0;

  /// Executes one step.
  virtual void step() = 0;
};

using TrajectoryExecutorPtr = std::shared_ptr<TrajectoryExecutor>;

} // namespace control
} // namespace aikido

#endif
