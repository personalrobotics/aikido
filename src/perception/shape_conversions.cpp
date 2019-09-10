#include <dart/common/Console.hpp>

#include "aikido/perception/shape_conversions.hpp"

namespace aikido {
namespace perception {

//==============================================================================
Eigen::Isometry3d convertROSPoseToEigen(const geometry_msgs::Pose& p)
{
  Eigen::Quaterniond eigen_quat(
      p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  Eigen::Vector3d eigen_trans(p.position.x, p.position.y, p.position.z);

  Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
  eigen_pose.rotate(eigen_quat);
  eigen_pose.translation() = eigen_trans;

  return eigen_pose;
}

//==============================================================================
Eigen::Isometry3d convertStampedTransformToEigen(const tf::StampedTransform& t)
{
  tf::Quaternion tf_quat(t.getRotation());
  Eigen::Quaterniond eigen_quat(
      tf_quat.getW(), tf_quat.getX(), tf_quat.getY(), tf_quat.getZ());
  Eigen::Vector3d eigen_trans(t.getOrigin());

  Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
  eigen_pose.rotate(eigen_quat);
  eigen_pose.translation() = eigen_trans;

  return eigen_pose;
}

} // namespace perception
} // namespace aikido
