#ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICSMOOTHER_HPP_
#define AIKIDO_PLANNER_PARABOLIC_PARABOLICSMOOTHER_HPP_

#include <Eigen/Dense>
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace parabolic {

constexpr double DEFAULT_TIMELIMT  = 3.0;
constexpr double DEFAULT_BLEND_RADIUS = 0.5;
constexpr int DEFAULT_BLEND_ITERATIONS = 4;
constexpr double DEFAULT_TOLERANCE = 1e-3;


/// Smooth a trajectory and apply shortcut
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _feasibilityCheck Check whether a position is feasible
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \param _timelimit The maximum time to allow for doing shortcut
/// (unit in second)
/// \return smoothed trajectory that satisfies acceleration constraints
std::unique_ptr<trajectory::Spline> doShortcut(
    const trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,    
    double _timelimit = DEFAULT_TIMELIMT,
    double _tolerance = DEFAULT_TOLERANCE,
    aikido::util::RNG::result_type _rngSeed = std::random_device{}());

/// Blend around waypoints in a trajectory using parabolic splines.
///
/// This function smooths `_inputTrajectory` by blending around
/// waypoints that have zero velocity using the following algorithm:
///
/// while _inputTrajectory has a waypoint with zero velocity:
/// - Construct the time-optimal parabolic spline from the state of the
///  trajectory at time `t - _blendRadius` to the state of the trajectory
///  at time `t + _blendRadius`.
/// - Check the validity of the segment against _feasabilityCheck.
/// - If the segment is valid, replace the portion of the trajectory between
///  times `t - _blendRadius` and `t + _blendRadius` with the segment.
///
/// If blending with _blendRadius fails to remove all waypoints with zero
/// velocity, the process described above repeats with half the initial
/// _blendRadius. Halving occurs _blendIteration times, with the final
/// iteration using a blend radius of
/// _blendRadius / std::pow(2, _blendIterations - 1).
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _feasibilityCheck Check whether a position is feasible
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \param _timelimit The maximum time to allow for doing shortcut
/// (unit in second)
/// \param _blendRadius the radius used in doing blend
/// \param _blendIterations the maximum iteration number in doing blend
/// \return smoothed trajectory that satisfies acceleration constraints
std::unique_ptr<trajectory::Spline> doBlend(
    const trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _blendRadius = DEFAULT_BLEND_RADIUS,
    int _blendIterations = DEFAULT_BLEND_ITERATIONS,
    double _tolerance = DEFAULT_TOLERANCE);

/// Smooth a trajectory and apply shortcut and blend
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _feasibilityCheck Check whether a position is feasible
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \param _timelimit The maximum time to allow for doing shortcut
/// (unit in second)
/// \param _blendRadius the radius used in doing blend
/// \param _blendIterations the maximum iteration number in doing blend
/// \return smoothed trajectory that satisfies acceleration constraints
std::unique_ptr<trajectory::Spline> doShortcutAndBlend(
    const trajectory::Spline& _inputTrajectory,
    aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit = DEFAULT_TIMELIMT,
    double _blendRadius = DEFAULT_BLEND_RADIUS,
    int _blendIterations = DEFAULT_BLEND_ITERATIONS,
    double _tolerance = DEFAULT_TOLERANCE,
    aikido::util::RNG::result_type _rngSeed = std::random_device{}()
    );

} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICSMOOTHER_HPP_
