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
#include <aikido/rviz/TSRMarker.hpp>
#include <aikido/rviz/pointers.hpp>
#include <aikido/trajectory/Trajectory.hpp>

namespace aikido {
namespace rviz {

AIKIDO_DECLARE_POINTERS(InteractiveMarkerViewer)

class InteractiveMarkerViewer
{
public:
  InteractiveMarkerViewer(
      const std::string& topicNamespace, const std::string& frameId);
  virtual ~InteractiveMarkerViewer();

  InteractiveMarkerViewer(const InteractiveMarkerViewer&) = delete;
  InteractiveMarkerViewer(const InteractiveMarkerViewer&&) = delete;
  InteractiveMarkerViewer& operator=(const InteractiveMarkerViewer&) = delete;

  interactive_markers::InteractiveMarkerServer& marker_server();

  SkeletonMarkerPtr addSkeleton(const dart::dynamics::SkeletonPtr& skeleton);

  FrameMarkerPtr addFrame(
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
  /// \return TSRMarkerPtr contains sampled frames of TSR.
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

  void setAutoUpdate(bool flag);
  void update();

protected:
  void autoUpdate();

  interactive_markers::InteractiveMarkerServer mMarkerServer;
  std::set<SkeletonMarkerPtr> mSkeletonMarkers;
  std::set<FrameMarkerPtr> mFrameMarkers;
  std::set<TrajectoryMarkerPtr> mTrajectoryMarkers;

  /// NameManager for name uniqueness of trajectories in the same
  /// InteractiveMarkerServer.
  dart::common::NameManager<trajectory::ConstTrajectoryPtr>
      mTrajectoryNameManager;

  std::atomic_bool mRunning;
  std::atomic_bool mUpdating;
  std::mutex mMutex;
  std::thread mThread;

  std::string mFrameId;
};

} // namespace rviz
} // namespace aikido

#endif
