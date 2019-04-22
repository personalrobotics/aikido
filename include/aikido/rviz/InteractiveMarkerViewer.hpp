#ifndef AIKIDO_RVIZ_INTERACTIVEMARKERVIEWER_HPP_
#define AIKIDO_RVIZ_INTERACTIVEMARKERVIEWER_HPP_

#include <atomic>
#include <set>
#include <thread>

#include <dart/common/NameManager.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/SmartPointer.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>

#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/TSRMarker.hpp>
#include <aikido/rviz/pointers.hpp>
#include <aikido/trajectory/Trajectory.hpp>

namespace aikido {
namespace rviz {

AIKIDO_DECLARE_POINTERS(InteractiveMarkerViewer)

class InteractiveMarkerViewer
{
public:
  /// Create an InteractiveMarkerViewer that reflects skeletons in a World.
  /// \param[in] topicNamespace ROS topic to publish marker updates to
  /// \param[in] frameId Base frame name
  /// \param[in] env World to update viewer with
  InteractiveMarkerViewer(
      const std::string& topicNamespace, const std::string& frameId, aikido::planner::WorldPtr env = nullptr);
  virtual ~InteractiveMarkerViewer();

  InteractiveMarkerViewer(const InteractiveMarkerViewer&) = delete;
  InteractiveMarkerViewer(const InteractiveMarkerViewer&&) = delete;
  InteractiveMarkerViewer& operator=(const InteractiveMarkerViewer&) = delete;

  interactive_markers::InteractiveMarkerServer& marker_server();

  /// Visualizes a Skeleton.
  /// \param skeleton Skeleton to add to the viewer
  /// \return Skeleton marker added to the viewer.
  SkeletonMarkerPtr addSkeletonMarker(const dart::dynamics::SkeletonPtr& skeleton);

  /// Visualizes a Frame as a cylinder.
  /// \param[in] frame Target DART frame.
  /// \param[in] length Length of the cylinderical frame (along z-axis).
  /// \param[in] thickness Radius (thickness in x and y axes) of cylinder.
  /// \param[in] alpha Opacity.
  /// \return Frame marker added to this viewer.
  FrameMarkerPtr addFrameMarker(
      dart::dynamics::Frame* frame,
      double length = 0.25,
      double thickness = 0.02,
      double alpha = 1.0);

  SkeletonMarkerPtr CreateSkeletonMarker(
      const dart::dynamics::SkeletonPtr& skeleton, const std::string& frameId);

  /// Visualizes a TSR.
  /// \param tsr TSR constraint
  /// \param nSamples Max number of samples to be used in visualization
  /// \param basename Basename for markers
  /// \return TSR marker that contains sampled frames of TSR.
  TSRMarkerPtr addTSRMarker(
      const constraint::dart::TSR& tsr,
      int nSamples = 10,
      const std::string& basename = "");

  /// Adds trajectory marker to this viewer.
  ///
  /// \param[in] trajectory C-space (or joint-space) trajectory.
  /// \param[in] skeleton Target DART meta skeleton that the C-space trajectory
  /// will be applied to compute the visualizing task-space trajectory.
  /// \param[in] frame Target DART frame where the trajectory of its origin
  /// will be visualized.
  /// \param[in] rgba Color and alpha of the visualized trajectory. Default is
  /// [RGBA: 0.75, 0.75, 0.75, 0.75].
  /// \param[in] thickness Thickness of the visualized trajectory. Default is
  /// 0.01.
  /// \param[in] numLineSegments Number of line segments in the visualized
  /// trajectory. Default is 16.
  /// \return Trajectory marker added to this viewer.
  TrajectoryMarkerPtr addTrajectoryMarker(
      trajectory::ConstTrajectoryPtr trajectory,
      dart::dynamics::MetaSkeletonPtr skeleton,
      const dart::dynamics::Frame& frame,
      const Eigen::Vector4d& rgba = Eigen::Vector4d::Constant(0.75),
      double thickness = 0.01,
      std::size_t numLineSegments = 16u);

  /// Set viewer auto-updating.
  /// \param[in] flag Whether to auto-update the viewer.
  void setAutoUpdate(bool flag);

  /// Update viewer with Skeletons from the World and existing markers.
  void update();

protected:
  /// Thread target for auto-updating the viewer.
  void autoUpdate();

  /// Interactive Marker Server.
  interactive_markers::InteractiveMarkerServer mMarkerServer;

  /// Mapping of Skeletons to SkeletonMarkers
  std::map<dart::dynamics::SkeletonPtr, SkeletonMarkerPtr> mSkeletonMarkers;

  /// Set of frame markers.
  std::set<FrameMarkerPtr> mFrameMarkers;

  /// Set of trajectory markers.
  std::set<TrajectoryMarkerPtr> mTrajectoryMarkers;

  /// NameManager for name uniqueness of trajectories in the same
  /// InteractiveMarkerServer.
  dart::common::NameManager<trajectory::ConstTrajectoryPtr>
      mTrajectoryNameManager;

  std::atomic_bool mRunning;
  std::atomic_bool mUpdating;
  mutable std::mutex mMutex;
  std::thread mThread;

  std::string mFrameId;

  /// World that automatically updates the viewer
  aikido::planner::WorldPtr mWorld;
};

} // namespace rviz
} // namespace aikido

#endif
