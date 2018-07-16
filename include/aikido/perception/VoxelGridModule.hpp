#ifndef AIKIDO_PERCEPTION_VOXELGRIDMODULE_HPP_
#define AIKIDO_PERCEPTION_VOXELGRIDMODULE_HPP_

#include <memory>
#include <string>

#include <dart/dynamics/dynamics.hpp>
#include <ros/ros.h>

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
      std::shared_ptr<dart::dynamics::VoxelGridShape> voxelGridShape = nullptr);

  /// Destructor
  virtual ~VoxelGridPerceptionModule() = default;

  /// Returns DART VoxelGridShape.
  std::shared_ptr<dart::dynamics::VoxelGridShape> getVoxelGridShape();

  /// Returns const DART VoxelGridShape.
  std::shared_ptr<const dart::dynamics::VoxelGridShape> getVoxelGridShape()
      const;

  /// Receives ROS msg and updates the voxel grid with the received ROS msg.
  ///
  /// \param[in] sensorOrigin Origin of sensor relative to frame.
  /// \param[in] inCoordinatesOf Reference frame, determines transform to be
  /// applied to point cloud and sensor origin.
  /// \param[in] timestamp Only detections more recent than this timestamp will
  /// be accepted. A timestamp of 0 greedily takes the first available message,
  /// and is the default behaviour.
  /// \return False if no points observed, or if no voxel grid is specified.
  /// True otherwise.
  bool update(
      const Eigen::Vector3d& sensorOrigin,
      const Eigen::Isometry3d& inCoordinatesOf,
      const ros::Duration& timeout);

  /// Receives ROS msg and updates the voxel grid with the received ROS msg.
  ///
  /// \param[in] sensorOrigin Origin of sensor relative to frame.
  /// \param[in] inCoordinatesOf Reference frame, determines transform to be
  /// applied to point cloud and sensor origin.
  /// \param[in] timestamp Only detections more recent than this timestamp will
  /// be accepted. A timestamp of 0 greedily takes the first available message,
  /// and is the default behaviour.
  /// \return False if no points observed, or if no voxel grid is specified.
  /// True otherwise.
  bool update(
      const Eigen::Vector3d& sensorOrigin,
      const dart::dynamics::Frame& inCoordinatesOf,
      const ros::Duration& timeout);

private:
  /// ROS node handle
  ros::NodeHandle mNodeHandle;

  /// ROS topic name
  std::string mPointCloudTopic;

  /// Voxel grid to update.
  std::shared_ptr<dart::dynamics::VoxelGridShape> mVoxelGridShape;

  /// Cache data to store point cloud data.
  octomap::Pointcloud mOctomapPointCloud;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_VOXELGRIDMODULE_HPP_
