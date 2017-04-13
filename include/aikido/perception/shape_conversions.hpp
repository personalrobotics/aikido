#ifndef AIKIDO_PERCEPTION_SHAPE_CONVERSIONS_HPP_
#define AIKIDO_PERCEPTION_SHAPE_CONVERSIONS_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>

namespace aikido {

namespace perception {

/// Convert a ROS Pose (position + quaternion) to a 4 x 4 pose matrix
/// \param[in] p the geometry_msgs::Pose pose value
/// return the 4 x 4 homogeneous matrix for the pose
Eigen::Isometry3d convertROSPoseToEigen(geometry_msgs::Pose const& p);

/// Convert a time-stamped ROS transform to a 4 x 4 pose matrix
/// \param[in] t the time-stamped ROS transform (position + quaternion)
/// \return the 4 x 4 homogeneous matrix corresponding to the time-stamped
/// transform
Eigen::Isometry3d convertStampedTransformToEigen(tf::StampedTransform const& t);

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_SHAPE_CONVERSIONS_HPP_
