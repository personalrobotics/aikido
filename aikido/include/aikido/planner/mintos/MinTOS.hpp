#ifndef AIKIDO_PLANNER_MINTOS_MINTOS_HPP_
#define AIKIDO_PLANNER_MINTOS_MINTOS_HPP_
#include <memory>
#include <aikido/constraint/Differentiable.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace mintos {

std::unique_ptr<trajectory::Spline> interpolateAndTimeOptimizeTrajectory(
  const trajectory::Interpolated& _inputTrajectory,
  const constraint::Differentiable& _constraint,
  const Eigen::VectorXd& _minVelocity,
  const Eigen::VectorXd& _maxVelocity,
  const Eigen::VectorXd& _minAcceleration,
  const Eigen::VectorXd& _maxAcceleration,
  double _constraintTolerance,
  double _interpolationTimestep);

} // namespace mintos
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_MINTOS_MINTOS_HPP_
