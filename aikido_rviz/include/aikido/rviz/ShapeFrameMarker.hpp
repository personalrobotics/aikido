#ifndef AIKIDO_RVIZ_SHAPEFRAMEMARKER_H_
#define AIKIDO_RVIZ_SHAPEFRAMEMARKER_H_

#include <boost/optional.hpp>
#include <dart/dynamics/dynamics.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "ResourceServer.hpp"

namespace aikido {
namespace rviz {

class ShapeFrameMarker
{
public:
  ShapeFrameMarker(
    ResourceServer *resourceServer,
    interactive_markers::InteractiveMarkerServer *markerServer,
    const std::string &name,
    const dart::dynamics::ShapeFrame *shapeFrame);

  ShapeFrameMarker(ShapeFrameMarker const &) = delete;
  ShapeFrameMarker &operator=(ShapeFrameMarker const &) = delete;

  virtual ~ShapeFrameMarker();

  bool update();

  void SetColor(Eigen::Vector4d const &color);
  void ResetColor();

private:
  ResourceServer *mResourceServer;
  interactive_markers::InteractiveMarkerServer *mMarkerServer;
  visualization_msgs::InteractiveMarker mInteractiveMarker;
  visualization_msgs::InteractiveMarkerControl *mVisualControl;

  const dart::dynamics::ShapeFrame *mShapeFrame;

  bool mExists;
  bool mForceUpdate;
  size_t mVersion;

  bool mShowVisual;
  bool mShowCollision;
  boost::optional<Eigen::Vector4d> mColor;
};

} // namespace rviz
} // namespace aikido

#endif
