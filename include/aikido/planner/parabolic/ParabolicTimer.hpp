#ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICTIMER_HPP_
#define AIKIDO_PLANNER_PARABOLIC_PARABOLICTIMER_HPP_

#include <Eigen/Dense>
#include "../../trajectory/Interpolated.hpp"
#include "../../trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace parabolic {

/// Computes the time-optimal timing of a trajectory consisting of a sequence
/// Geodesic interpolations between states under velocity and acceleration
/// bounds. The output is a parabolic spline, encoded in cubic polynomials,
/// that \b exactly follows the input path.
///
/// The output trajectory consists of a sequence of trapezoidal velocity
/// profiles that implement bang-bang control. This trajectory must stop at
/// each waypoint that introduces a velocity discontinuity to satisfy finite
/// acceleration bounds. You should consider using a blending or smoothing
/// algorithm, which does \b not follow the exact input path, if this behavior
/// is undesirable.
///
/// This function curently only supports \c RealVector, \c SO2, and compound
/// state spaces of those types. Additionally, this function requires that
/// \c _inputTrajectory to be interpolated using a \c GeodesicInterpolator.
///
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \return time optimal trajectory that satisfies acceleration constraints
std::unique_ptr<aikido::trajectory::Spline> computeParabolicTiming(
    const aikido::trajectory::Interpolated& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration);

/// Computes the time-optimal timing of a trajectory consisting of a linear
/// spline between states under velocity and acceleration
/// bounds. The output is a parabolic spline, encoded in cubic polynomials,
/// that \b exactly follows the input path.
///
/// The output trajectory consists of a sequence of trapezoidal velocity
/// profiles that implement bang-bang control. This trajectory must stop at
/// each waypoint that introduces a velocity discontinuity to satisfy finite
/// acceleration bounds. You should consider using a blending or smoothing
/// algorithm, which does \b not follow the exact input path, if this behavior
/// is undesirable.
///
/// This function curently only supports \c RealVector, \c SO2, and compound
/// state spaces of those types. Additionally, this function requires that
/// \c _inputTrajectory to be interpolated using a \c GeodesicInterpolator.
///
/// \param _inputTrajectory linear spline trajectory
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \return time optimal trajectory that satisfies acceleration constraints
std::unique_ptr<aikido::trajectory::Spline> computeParabolicTiming(
    const aikido::trajectory::Spline& _inputTrajectory,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration);

/// Convert an interpolated trajectory to a piecewise linear spline trajectory
/// This function requires the \c _inputTrajectory to use a \c
/// GeodesicInterpolator.
/// So the conversion exactly preserves the geometric path
/// \param _inputTrajectory interpolated trajectory
/// \return a spline trajectory
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const aikido::trajectory::Interpolated& _inputTrajectory);

} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICTIMER_HPP_
