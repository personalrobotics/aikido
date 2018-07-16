#include "aikido/perception/PointCloud.hpp"

#include <octomap_ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

namespace aikido {
namespace perception {

//=============================================================================
VoxelGridPerceptionModule::VoxelGridPerceptionModule(
    const ros::NodeHandle& nodeHandle,
    const std::string& pointCloudTopic,
    std::shared_ptr<dart::dynamics::VoxelGridShape> voxelGridShape)
  : mNodeHandle(nodeHandle)
  , mPointCloudTopic(pointCloudTopic)
  , mVoxelGridShape(std::move(voxelGridShape))
{
  // Do nothing
}

//==============================================================================
std::shared_ptr<dart::dynamics::VoxelGridShape>
VoxelGridPerceptionModule::getVoxelGridShape()
{
  return mVoxelGridShape;
}

//==============================================================================
std::shared_ptr<const dart::dynamics::VoxelGridShape>
VoxelGridPerceptionModule::getVoxelGridShape() const
{
  return mVoxelGridShape;
}

//==============================================================================
bool VoxelGridPerceptionModule::update(const ros::Duration& timeout)
{
  if (!mVoxelGridShape)
  {
    dtwarn << "[PointCloud] No DART VoxelGridShape is specified to update.";
    return false;
  }

  const auto pointCloud2Message
      = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          mPointCloudTopic, mNodeHandle, timeout);

  if (!pointCloud2Message)
  {
    dtwarn << "[PointCloud] Recieved null point cloud message: "
           << mPointCloudTopic << "\n";
    return false;
  }

  // Need to clear the cache becuase octomap::pointCloud2ToOctomap() appends
  // data.
  mOctomapPointCloud.clear();

  octomap::pointCloud2ToOctomap(*pointCloud2Message, mOctomapPointCloud);

  mVoxelGridShape->updateOccupancy(mOctomapPointCloud, octomap::point3d());

  return true;
}

} // namespace perception
} // namespace aikido
