#ifndef AIKIDO_PLANNER_MINTOS_MINTOS_HPP_
#define AIKIDO_PLANNER_MINTOS_MINTOS_HPP_
#include <memory>
#include "aikido/constraint/Differentiable.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace mintos {

// TODO(avk): Needs growth factor?
// TODO(avk): Does this need/require separation into smoother and timer?
std::unique_ptr<trajectory::Spline> interpolateAndTimeOptimizeTrajectory(
  const trajectory::Interpolated& inputTrajectory,
  const constraint::Differentiable& constraint,
  const Eigen::VectorXd& minVelocity,
  const Eigen::VectorXd& maxVelocity,
  const Eigen::VectorXd& minAcceleration,
  const Eigen::VectorXd& maxAcceleration,
  double constraintTolerance,
  double interpolationTimestep);

} // namespace mintos
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_MINTOS_MINTOS_HPP_
