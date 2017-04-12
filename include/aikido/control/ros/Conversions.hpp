#ifndef AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
#define AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
#include <memory>
#include <trajectory_msgs/JointTrajectory.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <map>

namespace aikido {
namespace control {
namespace ros {

/// Converts a ROS JointTrajectory into an aikido's Spline trajectory.
/// This method only handles single-DOF joints.
/// \param[in] space MetaSkeletonStateSpace for Spline trajectory.
//              Subspaces must be either 1D RnJoint or SO2Joint.
/// \param[in] jointTrajectory ROS JointTrajectory to be converted.
/// \return Spline trajectory.
std::unique_ptr<aikido::trajectory::Spline> toSplineJointTrajectory(
  const std::shared_ptr<
    aikido::statespace::dart::MetaSkeletonStateSpace>& space,
  const trajectory_msgs::JointTrajectory& jointTrajectory);

/// Converts Aikido Trajectory to ROS JointTrajectory.
/// Supports only 1D RnJoints and SO2Joints.
/// \param[in] trajectory Aikido trajectory to be converted.
/// \param[in] indexMap Mapping between trajectory's joints and ros joints.
/// \param[in] timestep Timestep between two consecutive waypoints.
trajectory_msgs::JointTrajectory toRosJointTrajectory(
  const aikido::trajectory::TrajectoryPtr& trajectory,
  const std::map<std::string, size_t>& indexMap, double timestep);

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
