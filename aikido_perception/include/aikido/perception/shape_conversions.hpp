/** 
 * @file shape_conversions.hpp
 * @author Shushman Choudhury
 * @date Apr 20, 2016
 * @brief The header for the utility file that has methods to convert
 * between ROS and Eigen Pose.
 */

#ifndef AIKIDO_PERCEPTION_SHAPE_CONVERSIONS_H
#define AIKIDO_PERCEPTION_SHAPE_CONVERSIONS_H

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

namespace aikido {

namespace perception {

Eigen::Isometry3d convertROSPoseToEigen(geometry_msgs::Pose const &p);
Eigen::Isometry3d convertStampedTransformToEigen(tf::StampedTransform const &t);


} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_SHAPE_CONVERSIONS_H