#ifndef AIKIDO_RVIZ_INTERACTIVEMARKERVIEWER_HPP_
#define AIKIDO_RVIZ_INTERACTIVEMARKERVIEWER_HPP_

#include <atomic>
#include <set>
#include <thread>

#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/SmartPointer.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>

#include <aikido/constraint/TSR.hpp>
#include <aikido/rviz/SmartPointers.hpp>
#include <aikido/rviz/TSRMarker.hpp>

namespace aikido {
namespace rviz {

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
      const aikido::constraint::TSR& tsr,
      int nSamples = 10,
      const std::string& basename = "");

  void setAutoUpdate(bool flag);
  void update();

protected:
  void autoUpdate();

  interactive_markers::InteractiveMarkerServer mMarkerServer;
  std::set<SkeletonMarkerPtr> mSkeletonMarkers;
  std::set<FrameMarkerPtr> mFrameMarkers;

  std::atomic_bool mRunning;
  std::atomic_bool mUpdating;
  std::mutex mMutex;
  std::thread mThread;

  std::string mFrameId;
};

} // namespace rviz
} // namespace aikido

#endif
