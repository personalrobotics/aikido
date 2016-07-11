#ifndef AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
#define AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
#include <memory>
#include <trajectory_msgs/JointTrajectory.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace control {
namespace ros {

std::unique_ptr<aikido::trajectory::Spline> convertJointTrajectory(
  const std::shared_ptr<
    aikido::statespace::dart::MetaSkeletonStateSpace>& space,
  const trajectory_msgs::JointTrajectory& jointTrajectory);

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
