#ifndef AIKIDO_RVIZ_INTERACTIVEMARKERVIEWER_H_
#define AIKIDO_RVIZ_INTERACTIVEMARKERVIEWER_H_
#include <atomic>
#include <set>
#include <thread>
#include <ros/ros.h>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/SmartPointer.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include "SmartPointers.hpp"
#include <aikido/constraint/TSR.hpp>

namespace aikido {
namespace rviz {

class InteractiveMarkerViewer {
public:
  InteractiveMarkerViewer(std::string const &topicNamespace);
  virtual ~InteractiveMarkerViewer();

  InteractiveMarkerViewer(InteractiveMarkerViewer const &) = delete;
  InteractiveMarkerViewer(InteractiveMarkerViewer const &&) = delete;
  InteractiveMarkerViewer &operator=(InteractiveMarkerViewer const &) = delete;

  interactive_markers::InteractiveMarkerServer &marker_server();

  SkeletonMarkerPtr addSkeleton(dart::dynamics::SkeletonPtr const &skeleton);

  FrameMarkerPtr addFrame(
    dart::dynamics::Frame *frame, double length = 0.25,
    double thickness = 0.02, double alpha = 1.0);

  SkeletonMarkerPtr CreateSkeletonMarker(
    dart::dynamics::SkeletonPtr const &skeleton);

  /// Visualizes tsr with at most n samples
  void visualizeTSR(const aikido::constraint::TSR& _tsr, int nSamples = 10);
  void cleanTSR();

  void setAutoUpdate(bool _flag);
  void update();

private:
  void autoUpdate();

  ros::NodeHandle mNodeHandle;
  interactive_markers::InteractiveMarkerServer mMarkerServer;
  std::set<SkeletonMarkerPtr> mSkeletonMarkers;
  std::set<FrameMarkerPtr> mFrameMarkers;
  std::vector<std::shared_ptr<dart::dynamics::SimpleFrame>> mSimpleFrames;

  std::atomic_bool mRunning;
  std::atomic_bool mUpdating;
  std::mutex mMutex;
  std::thread mThread;
};

} // namespace rviz
} // namespace aikido

#endif
