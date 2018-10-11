#ifndef AIKIDO_TRAJECTORY_UTIL_HPP_
#define AIKIDO_TRAJECTORY_UTIL_HPP_

#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace trajectory {

/// Converts an interpolated trajectory to a piecewise linear spline trajectory
/// This function requires the \c _inputTrajectory to use a \c
/// GeodesicInterpolator.
/// So the conversion exactly preserves the geometric path
/// \param[in] _inputTrajectory Interpolated trajectory
/// \return a spline trajectory
aikido::trajectory::UniqueSplinePtr convertToSpline(
    const aikido::trajectory::Interpolated& _inputTrajectory);

/// Converts a piecewise linear spline trajectory to an interpolated trajectory
/// using a given interpolator.
/// \param[in] traj Spline trajectory
/// \param[in] interpolator Interpolator used in connecting two ends of a
/// segment
/// \return an interpolated trajectory
aikido::trajectory::UniqueInterpolatedPtr convertToInterpolated(
    const aikido::trajectory::Spline& traj,
    aikido::statespace::ConstInterpolatorPtr& interpolator);

/// Concatenates two spline trajectories
/// This function converts two spline trajectories into geodesically
//  interpolated for concatenation, then converts the concatenated
/// interpolated to spline. The start state of the last segment in the first
/// trajectory is connected with the start of the first segment in the second
/// trajectory in concatenation.
///
/// \param[in] traj1 The first half spline trajectory
/// \param[in] traj2 The second half spline trajectory
/// \return the concatenated spline trajectory
aikido::trajectory::UniqueSplinePtr concatenate(
    const aikido::trajectory::Spline& traj1,
    const aikido::trajectory::Spline& traj2);

/// Finds the time of the closest state on a trajectory to a given
/// state.
///
/// \param[in] traj Input trajectory
/// \param[in] state Reference state
/// \param[in] timeStep Time step in finding the closest state
/// \return the time of the closest state on the input trajectory
double findTimeOfClosetStateOnTrajectory(
    const aikido::trajectory::Trajectory* traj,
    const Eigen::VectorXd& refenceState,
    double timeStep = 0.01);

/// Creates a partial Spline trajectory by a given start time.
///
/// \param[in] traj Original spline trajectory
/// \param[in] partialStartTime Start time of the new trajectory
/// \throw If \c partialStartTime in not in the interval defined by
/// the trajectory start time and end time.
/// \return the new partial trajectory
aikido::trajectory::UniqueSplinePtr createPartialTrajectory(
    const aikido::trajectory::Spline& traj, double partialStartTime);

} // namespace trajectory
} // namespace aikido

#endif
