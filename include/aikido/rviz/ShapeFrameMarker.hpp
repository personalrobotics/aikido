#ifndef AIKIDO_RVIZ_SHAPEFRAMEMARKER_HPP_
#define AIKIDO_RVIZ_SHAPEFRAMEMARKER_HPP_

#include <boost/optional.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>

#include "aikido/common/pointers.hpp"
#include "aikido/rviz/ResourceServer.hpp"

namespace aikido {
namespace rviz {

AIKIDO_DECLARE_POINTERS(ShapeFrameMarker)

class ShapeFrameMarker
{
public:
  ShapeFrameMarker(
      ResourceServer* resourceServer,
      interactive_markers::InteractiveMarkerServer* markerServer,
      const std::string& name,
      const dart::dynamics::ShapeFrame* shapeFrame,
      const std::string& frameId);

  ShapeFrameMarker(const ShapeFrameMarker&) = delete;
  ShapeFrameMarker& operator=(const ShapeFrameMarker&) = delete;

  virtual ~ShapeFrameMarker();

  bool update();

  void SetColor(const Eigen::Vector4d& color);
  void ResetColor();

private:
  ResourceServer* mResourceServer;
  interactive_markers::InteractiveMarkerServer* mMarkerServer;
  visualization_msgs::InteractiveMarker mInteractiveMarker;
  visualization_msgs::InteractiveMarkerControl* mVisualControl;

  const dart::dynamics::ShapeFrame* mShapeFrame;
  std::string mFrameId;

  bool mExists;
  bool mForceUpdate;
  std::size_t mVersion;

  bool mShowVisual;
  bool mShowCollision;
  boost::optional<Eigen::Vector4d> mColor;
};

} // namespace rviz
} // namespace aikido

#endif
