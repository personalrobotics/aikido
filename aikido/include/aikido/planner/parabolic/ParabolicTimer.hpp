#ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICSMOOTHER_HPP_
#define AIKIDO_PLANNER_PARABOLIC_PARABOLICSMOOTHER_HPP_
#include <Eigen/Dense>
#include "../../path/PiecewiseLinearTrajectory.hpp"
#include "../../path/SplineTrajectory2.hpp"

namespace aikido {
namespace planner {
namespace parabolic {

std::unique_ptr<path::SplineTrajectory2> computeParabolicTiming(
  const path::PiecewiseLinearTrajectory& _inputTrajectory,
  const Eigen::VectorXd& _maxVelocity,
  const Eigen::VectorXd& _maxAcceleration);

} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICSMOOTHER_HPP_
