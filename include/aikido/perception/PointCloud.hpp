#ifndef AIKIDO_PERCEPTION_POINTCLOUD_HPP_
#define AIKIDO_PERCEPTION_POINTCLOUD_HPP_

#include <memory>
#include <string>

#include <dart/dart.hpp>
#include <ros/ros.h>

namespace aikido {
namespace perception {

class PointCloud
{
public:
  PointCloud(
      const ros::NodeHandle& nodeHandle, const std::string& pointCloudTopic);

  virtual ~PointCloud() = default;

  bool updatePointCloud(const ros::Duration& timeout);

private:
  ros::NodeHandle mNodeHandle;

  std::string mPointCloudTopic;

  std::shared_ptr<dart::dynamics::VoxelShape> mVoxelShape;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_POINTCLOUD_HPP_
