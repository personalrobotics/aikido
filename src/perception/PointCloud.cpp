#include "aikido/perception/PointCloud.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <octomap_ros/conversions.h>

namespace aikido {
namespace perception {

//=============================================================================
PointCloud::PointCloud(
    const ros::NodeHandle& nodeHandle, const std::string& pointCloudTopic)
  : mNodeHandle(nodeHandle), mPointCloudTopic(pointCloudTopic)
{
  // Do nothing
}

//==============================================================================
bool PointCloud::updatePointCloud(const ros::Duration& timeout)
{
  auto pointCloud2Message
      = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          mPointCloudTopic, mNodeHandle, timeout);

  if (!pointCloud2Message)
  {
    dtwarn << "[PointCloud] Recieved null point cloud message: "
           << mPointCloudTopic << "\n";
    return false;
  }

  mOctomapPointCloud.clear();

  octomap::pointCloud2ToOctomap(*pointCloud2Message, mOctomapPointCloud);

  mVoxelGridShape->updateOccupancy(mOctomapPointCloud, octomap::point3d());

  return true;
}

} // namespace perception
} // namespace aikido
