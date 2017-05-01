#ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICTIMER_HPP_
#define AIKIDO_PLANNER_PARABOLIC_PARABOLICTIMER_HPP_

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

std::unique_ptr<trajectory::Spline> doShortcut(
    const trajectory::Spline* _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit = DEFAULT_TIMELIMT,
    bool _useVelocity = DEFAULT_USE_VELOCITY);

std::unique_ptr<trajectory::Spline> doBlend(
    const trajectory::Spline* _inputTrajectory,
    const aikido::constraint::TestablePtr _feasibilityCheck,
    const Eigen::VectorXd& _maxVelocity,
    const Eigen::VectorXd& _maxAcceleration,
    double _timelimit = DEFAULT_TIMELIMT,
    bool _useVelocity = DEFAULT_USE_VELOCITY,
    double _blendRadius = DEFAULT_BLEND_RADIUS,
    int _blendIterations = DEFAULT_BLEND_ITERATIONS);

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

#endif // ifndef AIKIDO_PLANNER_PARABOLIC_PARABOLICTIMER_HPP_
