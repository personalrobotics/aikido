#ifndef AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
#define AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_

#include <memory>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace control {
namespace ros {

/// Converts a ROS JointTrajectory into an aikido's Spline trajectory.
/// This method only handles single-DOF joints.
/// \param[in] space MetaSkeletonStateSpace for Spline trajectory.
//              Subspaces must be either R1Joint or SO2Joint.
/// \param[in] jointTrajectory ROS JointTrajectory to be converted.
/// \param[in] startPositions If empty, jointTrajectory must specify all joints
///             in the space. If non-empty, jointTrajectory only needs to
///             specify a subset of the joints in the space. The remaining
///             unspecified joint positions are filled with the corresponding
///             positions in startPositions, which must have the same size as
///             the space. Velocities and accelerations are filled with zero.
/// \return Spline trajectory.
std::unique_ptr<aikido::trajectory::Spline> toSplineJointTrajectory(
    const std::shared_ptr<aikido::statespace::dart::MetaSkeletonStateSpace>&
        space,
    const trajectory_msgs::JointTrajectory& jointTrajectory,
    const Eigen::VectorXd& startPositions);

/// Converts Aikido Trajectory to ROS JointTrajectory.
/// Supports only 1D RnJoints and SO2Joints.
/// \param[in] trajectory Aikido trajectory to be converted.
/// \param[in] timestep Timestep between two consecutive waypoints.
trajectory_msgs::JointTrajectory toRosJointTrajectory(
    const aikido::trajectory::TrajectoryPtr& trajectory, double timestep);

/// Converts Eigen VectorXd and joint names to JointState
/// \param[in] goalPositions The required positions for the fingers
/// \param[in] jointNames The corresponding names of the joints
/// \return The JointState message object
sensor_msgs::JointState positionsToJointState(
    const Eigen::VectorXd& goalPositions,
    const std::vector<std::string>& jointNames);

} // namespace ros
} // namespace control
} // namespace aikido

#endif // ifndef AIKIDO_CONTROL_ROS_CONVERSIONS_HPP_
