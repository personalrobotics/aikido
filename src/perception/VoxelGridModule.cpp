#ifdef DART_HAS_VOXELGRIDSHAPE

#include <chrono>

#include "aikido/perception/VoxelGridModule.hpp"

#include <octomap_ros/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/crop_box.h>

namespace aikido {
namespace perception {

//=============================================================================
VoxelGridPerceptionModule::VoxelGridPerceptionModule(
    const ros::NodeHandle& nodeHandle,
    const std::string& pointCloudTopic,
    const tf2_ros::Buffer& transformBuffer,
    const std::shared_ptr<aikido::planner::World> world,
    const double resolution,
    const std::string& worldFrame,
    const std::string& voxelSkeletonName)
  : mNodeHandle(nodeHandle)
  , mPointCloudTopic(pointCloudTopic)
  , mVoxelGridShape(std::make_shared<dart::dynamics::VoxelGridShape>(resolution))
  , mWorldFrame(worldFrame)
  , mVoxelSkeleton(dart::dynamics::Skeleton::create(voxelSkeletonName))
  , mTFBuffer(transformBuffer)
  , mWorld(std::move(world))
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
  catch (tf2::TransformException)
  {
    dtwarn << "[PointCloud] Could not find Transform from Static Reference Frame [" << mWorldFrame << "] to point cloud frame ["
           << pointCloud2Message->header.frame_id << "] \n" ;
    return false;
  }
  
  // Need to clear the cache becuase octomap::pointCloud2ToOctomap() appends
  // data.
  mOctomapPointCloud.clear();

  std::size_t numShapes = 0;
  std::vector<dart::math::BoundingBox> boundingBoxes;
  std::vector<Eigen::Isometry3d> transforms;
  for(std::size_t i = 0; i < mWorld->getNumSkeletons(); i++)
  {
    auto skeleton = mWorld->getSkeleton(i);
    for(std::size_t j = 0; j < skeleton->getNumBodyNodes(); j++)
    {
      numShapes += skeleton->getBodyNode(j)->getShapeNodesWith<dart::dynamics::CollisionAspect>().size();
    }
  }

  boundingBoxes.reserve(numShapes);
  transforms.reserve(numShapes);

  transforms.push_back(Eigen::Isometry3d::Identity());
  boundingBoxes.push_back(dart::math::BoundingBox(Eigen::Vector3d{-2.0, -2.0, -2.0}, Eigen::Vector3d{2.0, 2.0, 2.0}));

  for(std::size_t i = 0; i < mWorld->getNumSkeletons(); i++)
  {
    auto skeleton = mWorld->getSkeleton(i);
    if(skeleton->getName() == mVoxelSkeleton->getName())
      continue;
    for(std::size_t j = 0; j < skeleton->getNumBodyNodes(); j++)
    {
      auto shapeNodes = skeleton->getBodyNode(j)->getShapeNodesWith<dart::dynamics::CollisionAspect>();
      for(auto& shapeNode : shapeNodes)
      {
        transforms.push_back(shapeNode->getWorldTransform());
        boundingBoxes.push_back(shapeNode->getShape()->getBoundingBox());
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr saved_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pointCloud2Message, *saved_cloud);
  pcl::CropBox<pcl::PointXYZ> crop(false);
  
  crop.setInputCloud(saved_cloud);

  
  for(std::size_t i = 0; i < boundingBoxes.size(); i++)
  {
    if(i == 1)
      crop.setNegative(true);

    auto transform = Eigen::Affine3f::Identity();
    transform.matrix() = transforms[i].matrix().cast<float>();
    auto& min = boundingBoxes[i].getMin();
    auto& max = boundingBoxes[i].getMax();
    crop.setMin({min[0], min[1], min[2], 0});
    crop.setMax({max[0], max[1], max[2], 0});
    crop.setTransform(transform);
    crop.filter(*saved_cloud);


  }

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = saved_cloud->begin(); it != saved_cloud->end(); ++it){
    mOctomapPointCloud.push_back(it->x, it->y, it->z);
  }

  mVoxelGridShape->updateOccupancy(
      mOctomapPointCloud, worldToCameraTransform.translation(), worldToCameraTransform);

  return true;
}

} // namespace perception
} // namespace aikido

#endif // #ifdef DART_HAS_VOXELGRIDSHAPE
