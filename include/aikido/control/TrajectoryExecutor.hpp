#ifndef AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#include "TrajectoryResult.hpp"
#include <aikido/trajectory/Trajectory.hpp>
#include <future>

namespace aikido {
namespace control {

class TrajectoryExecutor
{
public:

  virtual ~TrajectoryExecutor() = default;

  /// Execute _traj and set future upon completion.
  /// \param _traj Trajectory to be executed.
  virtual std::future<void> execute(
    trajectory::TrajectoryPtr _traj) = 0;
};

using TrajectoryExecutorPtr = std::shared_ptr<TrajectoryExecutor>;

} // control
} // aikido

#endif
