#include "aikido/perception/PointCloud.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

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

  sensor_msgs::PointCloud pointCloudMessage;
  sensor_msgs::convertPointCloud2ToPointCloud(
      *pointCloud2Message, pointCloudMessage);

  const auto& points = pointCloudMessage.points;
  if (points.empty())
  {
    // TODO(JS): Print warnings?
    return false;
  }

  for (const auto& point : points)
  {
    std::cout << "point: (" << point.x << ", " << point.y << ", " << point.z
              << ")\n";
  }

  return true;
}

} // namespace perception
} // namespace aikido
