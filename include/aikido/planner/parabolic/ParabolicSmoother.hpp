#ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICSMOOTHER_HPP_
#define AIKIDO_PLANNER_PARABOLIC_PARABOLICSMOOTHER_HPP_

#include <Eigen/Dense>
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace parabolic {

static const double DEFAULT_TIMELIMT = 3.0;
static const bool DEFAULT_USE_VELOCITY = true;
static const double DEFAULT_BLEND_RADIUS = 0.5;
static const int DEFAULT_BLEND_ITERATIONS = 4;

/// Smooth a trajectory and apply shortcut
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _feasibilityCheck Check whether a position is feasible
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \param _timelimit The maximum time to allow for doing shortcut
/// \param _useVelocity whether velocity is considered in shortcut
/// \return smoothed trajectory that satisfies acceleration constraints
std::unique_ptr<trajectory::Spline> doShortcut(
    const trajectory::Interpolated& _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit = DEFAULT_TIMELIMT,
    bool _useVelocity = DEFAULT_USE_VELOCITY);

/// Smooth a trajectory and apply blend
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _feasibilityCheck Check whether a position is feasible
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \param _timelimit The maximum time to allow for doing shortcut
/// \param _useVelocity whether velocity is considered in shortcut
/// \param _blendRadius the radius used in doing blend
/// \param _blendIterations the maximum iteration number in doing blend
/// \return smoothed trajectory that satisfies acceleration constraints
std::unique_ptr<trajectory::Spline> doBlend(
    const trajectory::Interpolated& _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit = DEFAULT_TIMELIMT,
    bool _useVelocity = DEFAULT_USE_VELOCITY,
    double _blendRadius = DEFAULT_BLEND_RADIUS,
    int _blendIterations = DEFAULT_BLEND_ITERATIONS);

/// Smooth a trajectory and apply shortcut and blend
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _feasibilityCheck Check whether a position is feasible
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \param _timelimit The maximum time to allow for doing shortcut
/// \param _useVelocity whether velocity is considered in shortcut
/// \param _blendRadius the radius used in doing blend
/// \param _blendIterations the maximum iteration number in doing blend
/// \return smoothed trajectory that satisfies acceleration constraints
std::unique_ptr<trajectory::Spline> doShortcutAndBlend(
    const trajectory::Interpolated& _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit = DEFAULT_TIMELIMT,
    bool _useVelocity = DEFAULT_USE_VELOCITY,
    double _blendRadius = DEFAULT_BLEND_RADIUS,
    int _blendIterations = DEFAULT_BLEND_ITERATIONS);

/// Smooth a trajectory and apply shortcut
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _feasibilityCheck Check whether a position is feasible
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \param _timelimit The maximum time to allow for doing shortcut
/// \param _useVelocity whether velocity is considered in shortcut
/// \return smoothed trajectory that satisfies acceleration constraints
std::unique_ptr<trajectory::Spline> doShortcut(
    const trajectory::Spline* _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit = DEFAULT_TIMELIMT,
    bool _useVelocity = DEFAULT_USE_VELOCITY);

/// Smooth a trajectory and apply blend
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _feasibilityCheck Check whether a position is feasible
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \param _timelimit The maximum time to allow for doing shortcut
/// \param _useVelocity whether velocity is considered in shortcut
/// \param _blendRadius the radius used in doing blend
/// \param _blendIterations the maximum iteration number in doing blend
/// \return smoothed trajectory that satisfies acceleration constraints
std::unique_ptr<trajectory::Spline> doBlend(
    const trajectory::Spline* _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit = DEFAULT_TIMELIMT,
    bool _useVelocity = DEFAULT_USE_VELOCITY,
    double _blendRadius = DEFAULT_BLEND_RADIUS,
    int _blendIterations = DEFAULT_BLEND_ITERATIONS);

/// Smooth a trajectory and apply shortcut and blend
/// \param _inputTrajectory input piecewise Geodesic trajectory
/// \param _feasibilityCheck Check whether a position is feasible
/// \param _maxVelocity maximum velocity for each dimension
/// \param _maxAcceleration maximum acceleration for each dimension
/// \param _timelimit The maximum time to allow for doing shortcut
/// \param _useVelocity whether velocity is considered in shortcut
/// \param _blendRadius the radius used in doing blend
/// \param _blendIterations the maximum iteration number in doing blend
/// \return smoothed trajectory that satisfies acceleration constraints
std::unique_ptr<trajectory::Spline> doShortcutAndBlend(
    const trajectory::Spline* _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit = DEFAULT_TIMELIMT,
    bool _useVelocity = DEFAULT_USE_VELOCITY,
    double _blendRadius = DEFAULT_BLEND_RADIUS,
    int _blendIterations = DEFAULT_BLEND_ITERATIONS);

} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICSMOOTHER_HPP_
