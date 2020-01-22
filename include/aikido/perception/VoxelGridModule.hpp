#ifndef AIKIDO_PERCEPTION_VOXELGRIDMODULE_HPP_
#define AIKIDO_PERCEPTION_VOXELGRIDMODULE_HPP_

#ifdef DART_HAS_VOXELGRIDSHAPE

#include <memory>
#include <string>

#include <dart/dynamics/dynamics.hpp>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <aikido/planner/World.hpp>

namespace aikido {
namespace perception {

/// Perception module to take point cloud ROS msg and update voxel grid.
class VoxelGridPerceptionModule
{
public:
  /// Constructor
  ///
  /// \param[in] nodeHandle ROS node handle.
  /// \param[in] pointCloudTopic The topic name.
  /// \param[in] voxelGridShape Voxel grid to update.
  VoxelGridPerceptionModule(
    const ros::NodeHandle& nodeHandle,
    const std::string& pointCloudTopic,
    const tf2_ros::Buffer& transformBuffer,
    const std::shared_ptr<aikido::planner::World> world,
    const double resolution,
    const std::string& worldFrame = std::string("map"),
    const std::string& voxelSkeletonName = std::string("voxel_skeleton"));

  /// Destructor
  virtual ~VoxelGridPerceptionModule() = default;

  /// Returns DART Skeleton for Voxel.
  std::shared_ptr<dart::dynamics::Skeleton> getVoxelGridSkeleton() const;

  /// Returns const DART VoxelGridShape.
  std::shared_ptr<const dart::dynamics::VoxelGridShape> getVoxelGridShape()
      const;

  /// Receives ROS msg, looks up transfrom and updates the voxel grid with 
  /// the received ROS msg.
  ///
  /// \param[in] timeout The duration up to which to wait for the point cloud.
  /// \return False if no points observed, or if no voxel grid is specified.
  /// True otherwise.
  bool update(const ros::Duration& timeout);

private:
  /// ROS node handle
  ros::NodeHandle mNodeHandle;

  /// ROS topic name
  std::string mPointCloudTopic;

  /// Voxel grid to update.
  std::shared_ptr<dart::dynamics::VoxelGridShape> mVoxelGridShape;

  /// Cache data to store point cloud data.
  octomap::Pointcloud mOctomapPointCloud;

  /// Name of the static frame to be used as a reference
  std::string mWorldFrame;

  /// Skeleton containing the Voxel Shape
  std::shared_ptr<dart::dynamics::Skeleton> mVoxelSkeleton;

  /// Refernce to the transform buffer used for finding the pose of the camera
  const tf2_ros::Buffer& mTFBuffer;

  /// World containing objects to be substracted
  std::shared_ptr<aikido::planner::World> mWorld;
};

} // namespace perception
} // namespace aikido

#endif // DART_HAS_VOXELGRIDSHAPE

#endif // AIKIDO_PERCEPTION_VOXELGRIDMODULE_HPP_
