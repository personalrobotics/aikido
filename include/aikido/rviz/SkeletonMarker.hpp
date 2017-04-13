#ifndef AIKIDO_RVIZ_SKELETONMARKER_H_
#define AIKIDO_RVIZ_SKELETONMARKER_H_
#include <unordered_map>
#include <dart/dynamics/dynamics.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "BodyNodeMarker.hpp"
#include "ResourceServer.hpp"
#include "SmartPointers.hpp"

namespace aikido {
namespace rviz {

class SkeletonMarker
{
public:
  SkeletonMarker(
      ResourceServer* resourceServer,
      interactive_markers::InteractiveMarkerServer* markerServer,
      dart::dynamics::WeakSkeletonPtr const& skeleton);

  dart::dynamics::SkeletonPtr getSkeleton() const;
  std::vector<BodyNodeMarkerPtr> bodynode_markers() const;

  bool update();

  BodyNodeMarkerPtr GetBodyNodeMarker(dart::dynamics::BodyNode const* bodynode);

  void SetColor(Eigen::Vector4d const& color);
  void ResetColor();

private:
  dart::dynamics::WeakSkeletonPtr mSkeleton;
  ResourceServer* mResourceServer;
  interactive_markers::InteractiveMarkerServer* mMarkerServer;
  std::unordered_map<dart::dynamics::BodyNode const*, BodyNodeMarkerPtr>
      mBodyNodeMarkers;

  bool mHasColor;
  Eigen::Vector4d mColor;
};

} // namespace rviz
} // namespace aikido

#endif
