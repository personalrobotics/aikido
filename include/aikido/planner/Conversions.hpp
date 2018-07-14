#ifndef AIKIDO_PLANNER_CONVERSIONS_HPP_
#define AIKIDO_PLANNER_CONVERSIONS_HPP_

#include "aikido/planner/TrajectoryPostProcessor.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {

/// Converts a trajectory in the cartesian product space of SO(2) and R1 joints to
/// a trajectory in cartesian product space of strictly only R1 joints.
/// \param[in] space MetaSkeletonStateSpace for input trajectory.
///             Subspaces must be either R1Joint or SO2Joint.
/// \param[in] trajectory Trajectory to be converted.
/// \return Converted trajectory.
aikido::trajectory::TrajectoryPtr toRevoluteJointTrajectory(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& space,
    const aikido::trajectory::InterpolatedPtr trajectory);

} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_CONVERSIONS_HPP_
