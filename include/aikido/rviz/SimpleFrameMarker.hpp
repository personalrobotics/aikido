#ifndef AIKIDO_RVIZ_SIMPLEFRAMEMARKER_HPP_
#define AIKIDO_RVIZ_SIMPLEFRAMEMARKER_HPP_

#include <memory>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "aikido/common/pointers.hpp"
#include "ResourceServer.hpp"
#include "ShapeFrameMarker.hpp"

namespace aikido {
namespace rviz {

AIKIDO_DECLARE_POINTERS(SimpleFrameMarker)

class SimpleFrameMarker
{
public:
  SimpleFrameMarker(
      ResourceServer* resourceServer,
      interactive_markers::InteractiveMarkerServer* markerServer,
      const dart::dynamics::WeakSimpleFramePtr& frame,
      const std::string& frameId);

  ~SimpleFrameMarker();

  dart::dynamics::SimpleFramePtr getSimpleFrame() const;

  void SetColor(const Eigen::Vector4d& color);
  void ResetColor();

  bool update();

private:
  dart::dynamics::WeakSimpleFramePtr mSimpleFrame;

  ResourceServer* mResourceServer;
  interactive_markers::InteractiveMarkerServer* mMarkerServer;
  visualization_msgs::InteractiveMarker mInteractiveMarker;
  visualization_msgs::InteractiveMarkerControl* mVisualControl;
  std::string mFrameId;
  std::string mName;
  Eigen::Vector4d mColor;
  std::unique_ptr<ShapeFrameMarker> mShapeFrameMarker;

  bool mHasColor;
  std::string getName(const dart::dynamics::SimpleFrame& frame) const;

};

} // namespace rviz
} // namespace aikido

#endif // ifndef AIKIDO_RVIZ_SIMPLEFRAMEMARKER_HPP_
