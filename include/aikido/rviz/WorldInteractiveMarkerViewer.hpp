#ifndef AIKIDO_RVIZ_WORLDINTERACTIVEMARKERVIEWER_HPP_
#define AIKIDO_RVIZ_WORLDINTERACTIVEMARKERVIEWER_HPP_

#include <map>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/rviz/SmartPointers.hpp>

namespace aikido {
namespace rviz {

class WorldInteractiveMarkerViewer : public InteractiveMarkerViewer
{
public:
  /// Create an InteractiveMarkerViewer that reflects skeletons in a World.
  /// \param env World to update viewer with
  /// \param topicNamespace ROS topic to publish marker updates to
  /// \param frameId Base frame name
  WorldInteractiveMarkerViewer(
      aikido::planner::WorldPtr env,
      const std::string& topicNamespace,
      const std::string& frameId);
  virtual ~WorldInteractiveMarkerViewer();

  WorldInteractiveMarkerViewer(const WorldInteractiveMarkerViewer&) = delete;
  WorldInteractiveMarkerViewer(const WorldInteractiveMarkerViewer&&) = delete;
  WorldInteractiveMarkerViewer& operator=(const WorldInteractiveMarkerViewer&)
      = delete;

  /// Set viewer auto-updating
  /// \param flag Whether to auto-update the viewer
  void setAutoUpdate(bool flag);

  /// Update viewer with Skeletons from the World
  void update();

protected:
  /// Thread target for auto-updating the viewer
  void autoUpdate();

  /// Mapping of Skeleton names to SkeletonMarkers
  std::map<std::string, SkeletonMarkerPtr> mSkeletonMarkers;

  /// World that automatically updates the viewer
  aikido::planner::WorldPtr mWorld;
};

} // namespace rviz
} // namespace aikido

#endif
