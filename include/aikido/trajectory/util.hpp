#ifndef AIKIDO_TRAJECTORY_UTIL_HPP_
#define AIKIDO_TRAJECTORY_UTIL_HPP_

#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace trajectory {

/// Convert an interpolated trajectory to a piecewise linear spline trajectory
/// This function requires the \c _inputTrajectory to use a \c
/// GeodesicInterpolator.
/// So the conversion exactly preserves the geometric path
/// \param[in] _inputTrajectory interpolated trajectory
/// \return a spline trajectory
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const aikido::trajectory::Interpolated& _inputTrajectory);

/// Convert a piecewise linear spline trajectory to an interpolated trajectory
/// using a given interpolator.
/// \param[in] spline trajectory
/// \param[in] interpolator used in connecting two ends of a segment
/// \return an interpolated trajectory
std::unique_ptr<aikido::trajectory::Interpolated> convertToInterpolated(
    const aikido::trajectory::Spline& traj,
    const aikido::statespace::InterpolatorPtr interpolator);

/// Concatenate two spline trajectories
/// This function converts two spline trajectories into interpolated
/// trajectories
/// (geodesic interpolator) for concatenation, then convert the concatenated
/// interpolated to spline. The start state of the last segment in the first
/// trajectory is connected with the start of the first segment in the second
/// trajectory in concatenation.
///
/// \param[in] the first half spline trajectory
/// \param[in] the second half spline trajectory
/// \return the concatenated spline trajectory
std::unique_ptr<aikido::trajectory::Spline> concatenate(
    const aikido::trajectory::Spline& traj1,
    const aikido::trajectory::Spline& traj2);

/// Find the time of the closest state on a trajectory to a given
/// state.
///
/// \param[in] input trajectory
/// \param[in] reference state
/// \param[in] time step in finding the closest state
/// \return the time of the closest state on the input trajectory
double findTimeOfClosetStateOnTrajectory(
    const aikido::trajectory::Trajectory* traj,
    const Eigen::VectorXd& refenceState,
    double timeStep = 0.01);

/// Create a partial Spline trajectory by a given start time.
///
/// \param[in] original spline trajectory
/// \param[in] start time of the new trajectory
/// \return the new partial trajectory
std::unique_ptr<aikido::trajectory::Spline> createPartialTrajectory(
    const aikido::trajectory::Spline& traj, double partialStartTime);

} // namespace trajectory
} // namespace aikido

#endif
