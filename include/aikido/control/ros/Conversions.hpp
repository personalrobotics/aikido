#ifndef AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
#define AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
#include <memory>
#include <trajectory_msgs/JointTrajectory.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace control {
namespace ros {

/// Converts a ROS JointTrajectory into an aikido's Spline Trajectory.
/// \param[in] _space MetaSkeletonStateSpace for Spline trajectory.
/// \param[in] _jointTrajectory ROS JointTrajectory to be converted.
/// \return Spline trajectory.
std::unique_ptr<aikido::trajectory::Spline> convertJointTrajectory(
  const std::shared_ptr<
    aikido::statespace::dart::MetaSkeletonStateSpace>& _space,
  const trajectory_msgs::JointTrajectory& _jointTrajectory);

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_

