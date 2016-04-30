#ifndef AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#define AIKIDO_CONTROL_TRAJECTORYEXECUTOR_HPP_
#include "TrajectoryResult.hpp"
#include "../trajectory/Trajectory.hpp"
#include <future>

namespace aikido {
namespace control {

class TrajectoryExecutor
{
public:

  virtual ~TrajectoryExecutor() = default;

  virtual std::future<TrajectoryResultPtr> execute(
    trajectory::TrajectoryPtr _traj) = 0;
};

using TrajectoryExecutorPtr = std::shared_ptr<TrajectoryExecutor>;

} // control
} // aikido

#endif
