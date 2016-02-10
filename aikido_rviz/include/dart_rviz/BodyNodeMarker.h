#ifndef AIKIDO_RVIZ_BODYNODEMARKER_H_
#define AIKIDO_RVIZ_BODYNODEMARKER_H_
#include <dart/dynamics/dynamics.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "ResourceServer.h"

namespace aikido {
namespace rviz {

class BodyNodeMarker {
public:
  BodyNodeMarker(
    ResourceServer *resourceServer,
    interactive_markers::InteractiveMarkerServer *markerServer,
    dart::dynamics::WeakBodyNodePtr const &bodyNode);

  BodyNodeMarker(BodyNodeMarker const &) = delete;
  BodyNodeMarker &operator=(BodyNodeMarker const &) = delete;

  virtual ~BodyNodeMarker();

  bool update();

  void SetColor(Eigen::Vector4d const &color);
  void ResetColor();

private:
  dart::dynamics::WeakBodyNodePtr mBodyNode;
  dart::common::Connection mOnColShapeAdded;
  dart::common::Connection mOnColShapeRemoved;
  dart::common::Connection mOnStructuralChange;

  ResourceServer *mResourceServer;
  interactive_markers::InteractiveMarkerServer *mMarkerServer;
  visualization_msgs::InteractiveMarker mInteractiveMarker;
  visualization_msgs::InteractiveMarkerControl *mVisualControl;

  bool mExists;
  bool mGeometryDirty;
  std::string mName;

  bool mHasColor;
  Eigen::Vector4d mColor;

  std::string getName(dart::dynamics::BodyNode const &bodyNode);

  void updateName(dart::dynamics::BodyNode const &bodyNode,
                  std::string const &newName);
  void updateGeometry(dart::dynamics::BodyNode const &bodyNode);
  void updatePose(dart::dynamics::BodyNode const &bodyNode);

  void onColShapeAdded(dart::dynamics::BodyNode const *bodyNode,
                       dart::dynamics::ConstShapePtr shape);
  void onColShapeRemoved(dart::dynamics::BodyNode const *bodyNode,
                         dart::dynamics::ConstShapePtr shape);
  void onStructuralChange(dart::dynamics::BodyNode const *bodyNode);
};

} // namespace rviz
} // namespace aikido

#endif
