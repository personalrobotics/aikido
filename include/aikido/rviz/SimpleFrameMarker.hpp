#ifndef AIKIDO_RVIZ_SIMPLEFRAMEMARKER_HPP_
#define AIKIDO_RVIZ_SIMPLEFRAMEMARKER_HPP_

#include <memory>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "aikido/common/pointers.hpp"
#include "ResourceServer.hpp"

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

  void update();

private:
  dart::dynamics::WeakSimpleFramePtr mSimpleFrame;
  ResourceServer* mResourceServer;
  interactive_markers::InteractiveMarkerServer* mMarkerServer;

  bool mHasColor;
  std::string mFrameId;
  Eigen::Vector4d mColor;
};

} // namespace rviz
} // namespace aikido

#endif // ifndef AIKIDO_RVIZ_SIMPLEFRAMEMARKER_HPP_
