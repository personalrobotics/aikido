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
      interactive_markers::InteractiveMarkerServer* markerServer,
      const std::string& frameId,
      const std::string& markerName,
      const dart::dynamics::SimpleFramePtr frame,
      const Eigen::Vector4d& rgba = Eigen::Vector4d::Constant(0.75));

  ~SimpleFrameMarker();

  dart::dynamics::SimpleFramePtr getSimpleFrame() const;

  /// Sets or updates color (RGB) of visualized simple frame.
  void setColor(const Eigen::Vector3d& rgb);

  /// Returns color (RGB) of visualized simple frame.
  Eigen::Vector3d getColor() const;

  /// Sets or updates RGBA of visualized simple frame.
  void setRGBA(const Eigen::Vector4d& rgb);

  /// Returns RGBA of visualized simple frame.
  Eigen::Vector4d getRBGA() const;

  /// Updates this marker.
  ///
  /// This function should be called after the properties are changed (e.g.,
  /// frame position, color, size to reflect changes in RViz accordingly.
  void update();

private:
  /// Returns marker.
  visualization_msgs::Marker& getMarker();

  /// Returns const marker.
  const visualization_msgs::Marker& getMarker() const;

  /// RViz marker server.
  interactive_markers::InteractiveMarkerServer* mMarkerServer;

  /// RViz interactive marker.
  visualization_msgs::InteractiveMarker mInteractiveMarker;

  /// Frame name of RViz interactive marker.
  std::string mFrameId;

  // Simple Frame associated with the marker
  dart::dynamics::SimpleFramePtr mSimpleFrame;

  /// Whether the associated RViz maker needs to be updated.
  bool mNeedUpdate;

};

} // namespace rviz
} // namespace aikido

#endif // ifndef AIKIDO_RVIZ_SIMPLEFRAMEMARKER_HPP_
