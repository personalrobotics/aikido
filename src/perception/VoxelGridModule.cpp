#ifdef DART_HAS_VOXELGRIDSHAPE

#include "aikido/perception/VoxelGridModule.hpp"

#include <octomap_ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace aikido {
namespace perception {

//=============================================================================
VoxelGridPerceptionModule::VoxelGridPerceptionModule(
    const ros::NodeHandle& nodeHandle,
    const std::string& pointCloudTopic,
    const tf2_ros::Buffer& transformBuffer,
    const double resolution,
    const std::string& worldFrame,
    const std::string& voxelSkeletonName)
  : mNodeHandle(nodeHandle)
  , mPointCloudTopic(pointCloudTopic)
  , mVoxelGridShape(std::make_shared<dart::dynamics::VoxelGridShape>(resolution))
  , mWorldFrame(worldFrame)
  , mVoxelSkeleton(dart::dynamics::Skeleton::create(voxelSkeletonName))
  , mTFBuffer(transformBuffer)
{
  using namespace dart::dynamics;

  FreeJoint::Properties properties;
  properties.mName = "Base_Joint";
  BodyNodePtr bn = mVoxelSkeleton->createJointAndBodyNodePair<FreeJoint>(nullptr, properties,
            BodyNode::AspectProperties("Base")).second;
  bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(mVoxelGridShape);

}

//==============================================================================
dart::dynamics::SkeletonPtr VoxelGridPerceptionModule::getVoxelGridSkeleton() const
{
  return mVoxelSkeleton;
}

//==============================================================================
std::shared_ptr<const dart::dynamics::VoxelGridShape>
VoxelGridPerceptionModule::getVoxelGridShape() const
{
  return mVoxelGridShape;
}

//==============================================================================
bool VoxelGridPerceptionModule::update(
    const ros::Duration& timeout)
{
  const auto pointCloud2Message
      = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
          mPointCloudTopic, mNodeHandle, timeout);

  if (!pointCloud2Message)
  {
    dtwarn << "[PointCloud] Recieved null point cloud message: "
           << mPointCloudTopic << "\n";
    return false;
  }

  Eigen::Isometry3d worldToCameraTransform;

  try
  {
    auto worldToCameraTF = mTFBuffer.lookupTransform(mWorldFrame, pointCloud2Message->header.frame_id, ros::Time(0));
    
    geometry_msgs::PoseStamped worldToCameraPose;
    worldToCameraPose.pose.position.x    = 0;
    worldToCameraPose.pose.position.y    = 0;
    worldToCameraPose.pose.position.z    = 0;
    worldToCameraPose.pose.orientation.x = 0;
    worldToCameraPose.pose.orientation.y = 0;
    worldToCameraPose.pose.orientation.z = 0;
    worldToCameraPose.pose.orientation.w = 1;

    tf2::doTransform(worldToCameraPose, worldToCameraPose, worldToCameraTF);
    worldToCameraTransform = Eigen::Translation3d(worldToCameraPose.pose.position.x, worldToCameraPose.pose.position.y,
                                                  worldToCameraPose.pose.position.z)
                            *Eigen::Quaterniond(worldToCameraPose.pose.orientation.w, worldToCameraPose.pose.orientation.x,
                                                worldToCameraPose.pose.orientation.y, worldToCameraPose.pose.orientation.z);

  }
  catch (tf2::LookupException)
  {
    dtwarn << "[PointCloud] Could not find Transform from Static Reference Frame [" << mWorldFrame << "] to point cloud frame ["
           << pointCloud2Message->header.frame_id << "] \n" ;
    return false;
  }

  // Need to clear the cache becuase octomap::pointCloud2ToOctomap() appends
  // data.
  mOctomapPointCloud.clear();

  octomap::pointCloud2ToOctomap(*pointCloud2Message, mOctomapPointCloud);

  mVoxelGridShape->updateOccupancy(
      mOctomapPointCloud, worldToCameraTransform.translation(), worldToCameraTransform);

  return true;
}

} // namespace perception
} // namespace aikido

#endif // #ifdef DART_HAS_VOXELGRIDSHAPE
