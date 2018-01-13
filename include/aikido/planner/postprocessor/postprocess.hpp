#ifndef AIKIDO_PLANNER_POSTPROCESSOR_POSTPROCESS_HPP_
#define AIKIDO_PLANNER_POSTPROCESSOR_POSTPROCESS_HPP_

#include "../../common/RNG.hpp"
#include "../../constraint/Testable.hpp"
#include "../../statespace/StateSpace.hpp"
#include "../../trajectory/Interpolated.hpp"
#include "../../trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace postprocessor {
namespace postprocess {
/// Perform parabolic retiming on an interpolated trajectory
/// \param _inputTraj The untimed trajectory to retime
/// \return The timed trajectory subject to limits
std::unique_ptr<aikido::trajectory::Spline> timeTrajectory(
    const aikido::trajectory::InterpolatedPtr& _inputTraj,
    const Eigen::VectorXd& velocityLimit,
    const Eigen::VectorXd& accelerationLimits);

/// Perform parabolic smoothing on trajectory
/// \param _inputTraj The untimed trajectory for the arm to smooth.
/// \param _enableShortcut Whether shortcutting is used in smoothing.
/// \param _enableBlend Whether blending is used in smoothing.
/// \param _collisionTestable Testable constraint to check for collision.
/// \param _rng Random number generator.
/// \param _shortcutTimelimit Timelimit for shortcutting. It is ineffective
/// when _enableShortcut is false.
/// \param _blendRadius Blend radius for blending. It is ineffective
/// when _enableBlend is false.
/// \param _blendIterations Blend iterations for blending. It is
/// ineffective when _enableBlend is false.
/// \return The smoothed trajectory subject to limits
std::unique_ptr<aikido::trajectory::Spline> smoothTrajectory(
    const aikido::trajectory::InterpolatedPtr& _inputTraj,
    bool _enableShortcut,
    bool _enableBlend,
    const aikido::constraint::TestablePtr& _collisionTestable,
    std::unique_ptr<aikido::common::RNG> _rng,
    const Eigen::VectorXd& _velocityLimit,
    const Eigen::VectorXd& _accelerationLimits,
    double _smootherFeasibilityCheckResolution,
    double _smootherFeasibilityApproxTolerance,
    double _shortcutTimelimit = 0.6,
    double _blendRadius = 0.4,
    int _blendIterations = 1);

} // namespace postprocess
} // namespace postprocessor
} // namespcae planner
} // namespace aikido

#endif // AIKIDO_PLANNER_POSTPROCESSOR_POSTPROCESS_HPP_
