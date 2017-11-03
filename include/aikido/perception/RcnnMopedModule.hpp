#ifndef AIKIDO_PERCEPTION_RCNNMOPEDMODULE_HPP_
#define AIKIDO_PERCEPTION_RCNNMOPEDMODULE_HPP_

#include <string>
#include <dart/dart.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <aikido/perception/PerceptionModule.hpp>

namespace aikido {
namespace perception {

class RcnnMopedModule : public PerceptionModule
{
public:
  RcnnMopedModule(
      ros::NodeHandle nodeHandle,
      std::string markerTopic,
      dart::common::ResourceRetrieverPtr resourceRetriever,
      std::string canUri,
      std::string referenceFrameId,
      dart::dynamics::Frame* referenceLink);

  virtual ~RcnnMopedModule() = default;

  // Documentation inherited
  bool detectObjects(
      const aikido::planner::WorldPtr& env,
      ros::Duration timeout = ros::Duration(0.0),
      ros::Time timestamp = ros::Time(0.0)) override;

private:
  ros::NodeHandle mNodeHandle;

  std::string mMarkerTopic;

  std::string mReferenceFrameId;

  dart::common::ResourceRetrieverPtr mResourceRetriever;

  std::string mCanUri;

  dart::dynamics::Frame* mReferenceLink;

  tf::TransformListener mTfListener;
};

}  // namespace perception
}  // namespace aikido
#endif  // AIKIDO_PERCEPTION_RCNNMOPEDMODULE_HPP_
