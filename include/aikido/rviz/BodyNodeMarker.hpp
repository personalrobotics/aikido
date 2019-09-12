#ifndef AIKIDO_RVIZ_BODYNODEMARKER_HPP_
#define AIKIDO_RVIZ_BODYNODEMARKER_HPP_

#include <unordered_map>

#include <dart/dynamics/dynamics.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>

#include "aikido/common/pointers.hpp"
#include "aikido/rviz/ResourceServer.hpp"
#include "aikido/rviz/ShapeFrameMarker.hpp"

namespace aikido {
namespace rviz {

AIKIDO_DECLARE_POINTERS(BodyNodeMarker)

class BodyNodeMarker
{
public:
  BodyNodeMarker(
      ResourceServer* resourceServer,
      interactive_markers::InteractiveMarkerServer* markerServer,
      const dart::dynamics::WeakBodyNodePtr& bodyNode,
      const std::string& frameId);

  BodyNodeMarker(const BodyNodeMarker&) = delete;
  BodyNodeMarker& operator=(const BodyNodeMarker&) = delete;

  virtual ~BodyNodeMarker() = default;

  bool update();

  void SetColor(const Eigen::Vector4d& color);
  void ResetColor();

private:
  using ShapeFrameMarkerMap = std::
      map<const dart::dynamics::ShapeNode*, std::unique_ptr<ShapeFrameMarker>>;

  dart::dynamics::WeakBodyNodePtr mBodyNode;
  dart::common::Connection mOnStructuralChange;

  ResourceServer* mResourceServer;
  interactive_markers::InteractiveMarkerServer* mMarkerServer;
  visualization_msgs::InteractiveMarker mInteractiveMarker;
  visualization_msgs::InteractiveMarkerControl* mVisualControl;
  std::string mFrameId;

  std::string mName;
  ShapeFrameMarkerMap mShapeFrameMarkers;

  std::string getName(const dart::dynamics::BodyNode& bodyNode) const;
};

} // namespace rviz
} // namespace aikido

#endif
