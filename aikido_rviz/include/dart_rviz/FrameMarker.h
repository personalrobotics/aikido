#ifndef DART_INTERACTIVEMARKER_FRAMEMARKER_H_
#define DART_INTERACTIVEMARKER_FRAMEMARKER_H_
#include <memory>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>

namespace dart {


namespace dynamics {

class Frame;

} // namespace dynamics


namespace rviz {

class FrameMarker {
public:
  FrameMarker(
    interactive_markers::InteractiveMarkerServer *markerServer,
    dart::dynamics::Frame *frame,
    double length = 0.25,
    double thickness = 0.02,
    double alpha = 1.0);
  ~FrameMarker();

  void update();

private:
  interactive_markers::InteractiveMarkerServer *mMarkerServer;
  visualization_msgs::InteractiveMarker mInteractiveMarker;

  dart::dynamics::Frame *mFrame;
};

} // namespace rviz
} // namespace dart

#endif // ifndef DART_INTERACTIVEMARKER_FRAMEMARKER_H_
