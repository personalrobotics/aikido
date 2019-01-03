#ifndef AIKIDO_TRAJECTORY_UTIL_HPP_
#define AIKIDO_TRAJECTORY_UTIL_HPP_

#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace trajectory {

/// Converts an interpolated trajectory to a piecewise linear spline trajectory.
///
/// This function requires the \c _inputTrajectory to use a \c
/// GeodesicInterpolator. So the conversion exactly preserves the geometric
/// path.
///
/// \param[in] inputTrajectory Interpolated trajectory
/// \return A spline trajectory
UniqueSplinePtr convertToSpline(const Interpolated& inputTrajectory);

/// Converts a piecewise linear spline trajectory to an interpolated trajectory
/// using a given interpolator.
///
/// \param[in] traj Spline trajectory
/// \param[in] interpolator Interpolator used in connecting two ends of a
/// segment
/// \return An interpolated trajectory
UniqueInterpolatedPtr convertToInterpolated(
    const Spline& traj, statespace::ConstInterpolatorPtr interpolator);

/// Concatenates two interpolated trajectories.
///
/// This function concatenates two interpolated trajectories into one
/// interpolated trajectory. The start state of the last segment in the first
/// trajectory is connected with the start of the first segment in the second
/// trajectory in concatenation. For example, concatenating trajectory a:
/// [wp1(t=1), wp2(t=2.1), wp3(t=4)] and trajectory b: [wp4(t=2), wp5(t=7)]
/// gets a new trajectory: [wp1(t=1), wp2(t=2.1), wp4(t=4), wp5(t=9)].
/// It gaurantees that the new duration is the sum of the durations of the two.
/// wp3' is dervied by merging wp3 and wp4, which connects the start of wp3 and
/// the end of wp4. The state spaces of two trajectories should be the same.
///
/// \param[in] traj1 The first half interpolated trajectory
/// \param[in] traj2 The second half interpolated trajectory
/// \return The concatenated interpolated trajectory
UniqueInterpolatedPtr concatenate(
    const Interpolated& traj1, const Interpolated& traj2);

/// Concatenates two spline trajectories.
///
/// This function converts two spline trajectories into geodesically
/// interpolated for concatenation, then converts the concatenated interpolated
/// to spline. The state spaces of two trajectories should be the same.
///
/// \param[in] traj1 The first half spline trajectory
/// \param[in] traj2 The second half spline trajectory
/// \return The concatenated spline trajectory
UniqueSplinePtr concatenate(const Spline& traj1, const Spline& traj2);

/// Finds the time of the closest state on a trajectory to a given state.
///
/// This function checks discrete states on [t0, t1, ..., tn] where t0 and tn
/// are the start time and end time of the trajectory, respectively. The two
/// endpoints are always included in the check. If two states are equidistant to
/// the reference state, then the first one along the time-dimension will be
/// returned.
///
/// \param[in] traj Input trajectory
/// \param[in] referenceState Reference state
/// \param[in] timeStep Time step in finding the closest state
/// \return The time of the closest state on the input trajectory
double findTimeOfClosestStateOnTrajectory(
    const Trajectory& traj,
    const Eigen::VectorXd& referenceState,
    double timeStep = 0.01);

/// Retrieves part of a given spline trajectory
///
/// Given a spline trajectory \c traj between [startTime, endTime] and a time
/// point \c partialStartTime, such that startTime <= \c partialStartTime <=
/// endTime, retrieves the part of \c traj between \c partialStartTime and
/// endTime. The retrieved spline is shifted in time to begin at startTime
/// instead of at \c partialStartTime.
///
/// \param[in] traj Original spline trajectory
/// \param[in] partialStartTime Start time of the new trajectory
/// \throw If \c partialStartTime is not in the interval defined by the
/// trajectory start time and end time.
/// \return The new partial trajectory
UniqueSplinePtr createPartialTrajectory(
    const Spline& traj, double partialStartTime);

/// Converts an interpolated trajectory in the cartesian product space of SO(2)
/// and R1 joints
/// to a trajectory in cartesian product space of strictly only R1 joints.
/// \param[in] space MetaSkeletonStateSpace for input trajectory.
/// \param[in] trajectory Trajectory to be converted.
/// \return Converted trajectory.
aikido::trajectory::ConstInterpolatedPtr toR1JointTrajectory(
    aikido::statespace::ConstStateSpacePtr& space,
    aikido::trajectory::ConstInterpolatedPtr& trajectory);

/// Converts a spline trajectory in the cartesian product space of SO(2) and R1
/// joints
/// to a trajectory in cartesian product space of strictly only R1 joints.
/// \param[in] space MetaSkeletonStateSpace for input trajectory.
/// \param[in] trajectory Trajectory to be converted.
/// \return Converted trajectory.
aikido::trajectory::ConstSplinePtr toR1JointTrajectory(
    aikido::statespace::ConstStateSpacePtr& space,
    aikido::trajectory::ConstSplinePtr& trajectory);

} // namespace trajectory
} // namespace aikido

#endif // AIKIDO_TRAJECTORY_UTIL_HPP_
