#ifndef AIKIDO_RVIZ_SKELETONMARKER_HPP_
#define AIKIDO_RVIZ_SKELETONMARKER_HPP_

#include <unordered_map>
#include <dart/dynamics/dynamics.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "BodyNodeMarker.hpp"
#include "ResourceServer.hpp"
#include "aikido/common/pointers.hpp"

namespace aikido {
namespace rviz {

AIKIDO_DECLARE_POINTERS(SkeletonMarker)

class SkeletonMarker
{
public:
  SkeletonMarker(
      ResourceServer* resourceServer,
      interactive_markers::InteractiveMarkerServer* markerServer,
      const dart::dynamics::WeakSkeletonPtr& skeleton,
      const std::string& frameId);

  dart::dynamics::SkeletonPtr getSkeleton() const;
  std::vector<BodyNodeMarkerPtr> bodynode_markers() const;

  bool update();

  BodyNodeMarkerPtr GetBodyNodeMarker(dart::dynamics::BodyNode const* bodynode);

  void SetColor(const Eigen::Vector4d& color);
  void ResetColor();

private:
  dart::dynamics::WeakSkeletonPtr mSkeleton;
  ResourceServer* mResourceServer;
  interactive_markers::InteractiveMarkerServer* mMarkerServer;
  std::unordered_map<dart::dynamics::BodyNode const*, BodyNodeMarkerPtr>
      mBodyNodeMarkers;

  bool mHasColor;
  std::string mFrameId;
  Eigen::Vector4d mColor;
};

} // namespace rviz
} // namespace aikido

#endif
