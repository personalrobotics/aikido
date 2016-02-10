#ifndef DART_INTERACTIVEMARKER_INTERACTIVEMARKERVIEWER_H_
#define DART_INTERACTIVEMARKER_INTERACTIVEMARKERVIEWER_H_
#include <atomic>
#include <set>
#include <thread>
#include <ros/ros.h>
#include <dart/dynamics/Frame.h>
#include <dart/dynamics/SmartPointer.h>
#include <interactive_markers/interactive_marker_server.h>
#include "SmartPointers.h"

namespace dart {
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

  void setAutoUpdate(bool _flag);
  void update();

private:
  void autoUpdate();

  ros::NodeHandle mNodeHandle;
  interactive_markers::InteractiveMarkerServer mMarkerServer;
  std::set<SkeletonMarkerPtr> mSkeletonMarkers;
  std::set<FrameMarkerPtr> mFrameMarkers;

  std::atomic_bool mRunning;
  std::atomic_bool mUpdating;
  std::mutex mMutex;
  std::thread mThread;
};

} // namespace rviz
} // namespace dart

#endif
