#ifndef AIKIDO_RVIZ_WORLDINTERACTIVEMARKERVIEWER_HPP_
#define AIKIDO_RVIZ_WORLDINTERACTIVEMARKERVIEWER_HPP_

#include <map>
#include <dart/simulation/World.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/rviz/SmartPointers.hpp>

namespace aikido {
namespace rviz {

class WorldInteractiveMarkerViewer : public InteractiveMarkerViewer
{
public:
  WorldInteractiveMarkerViewer(
      dart::simulation::WorldPtr env,
      const std::string& topicNamespace,
      const std::string& frameId);
  virtual ~WorldInteractiveMarkerViewer();

  WorldInteractiveMarkerViewer(const WorldInteractiveMarkerViewer&) = delete;
  WorldInteractiveMarkerViewer(const WorldInteractiveMarkerViewer&&) = delete;
  WorldInteractiveMarkerViewer& operator=(const WorldInteractiveMarkerViewer&)
      = delete;

  void setAutoUpdate(bool flag);
  void update();

protected:
  void autoUpdate();

  std::map<std::string, SkeletonMarkerPtr> mSkeletonMarkers;
  dart::simulation::WorldPtr mWorld;
};

} // namespace rviz
} // namespace aikido

#endif
