#ifndef AIKIDO_PERCEPTION_APRILTAGSMODULE_HPP_
#define AIKIDO_PERCEPTION_APRILTAGSMODULE_HPP_

#include <memory>
#include <string>
#include <dart/dart.hpp>
#include <ros/forwards.h>
#include <ros/ros.h>
#include <ros/single_subscriber_publisher.h>
#include <tf/transform_listener.h>
#include <aikido/perception/AprilTagsDatabase.hpp>
#include <aikido/perception/PerceptionModule.hpp>
#include <aikido/common/CatkinResourceRetriever.hpp>
#include <aikido/io/yaml.hpp>

namespace aikido {
namespace perception {

/// Instantiates the \c PerceptionModule specifically for AprilTags.
///
/// It receives input from an April Tags detector running in a separate process,
/// by subscribing to a visualization_msg::Marker ROS topic published by the
/// AprilTags node. It uses a \c AprilTagsDatabase to resolve each marker to a
/// \c dart::common::Uri, and updates the environment which is a list of \c
/// dart::dynamics::SkeletonPtr.
class AprilTagsModule : public PerceptionModule
{
public:
  /// Construct an AprilTags receiver that subscribes to the specified topic
  /// where AprilTag information is being published as a
  /// \c visualization_msg::MarkerArray, uses a database loader for
  /// configuration information related to tags and obtains the desired
  /// transformation frame for the object pose.
  ///
  ///	\param[in] node The node handle to be passed to the detector
  ///	\param[in] markerTopic The name of the topic on which april tags
  /// information is being published
  ///	\param[in] configData The pointer to some configuration data loader
  ///	\param[in] resourceRetriever A DART retriever for resources related to
  /// config files and models and so on
  ///	\param[in] destinationFrame The desired TF for the detections
  ///	\param[in] referenceLink A link on the robot with respect to which the
  /// pose is transformed
  AprilTagsModule(
      ros::NodeHandle node,
      std::string markerTopic,
      std::shared_ptr<AprilTagsDatabase> configData,
      dart::common::ResourceRetrieverPtr resourceRetriever,
      std::string destinationFrame,
      dart::dynamics::Frame* referenceLink);

  virtual ~AprilTagsModule() = default;

  // Documentation inherited
  bool detectObjects(
      const dart::simulation::WorldPtr& env,
      ros::Duration timeout = ros::Duration(0.0),
      ros::Time timestamp = ros::Time(0.0)) override;

private:
  /// The name of the ROS topic to read marker info from
  std::string mMarkerTopic;

  /// The desired reference frame for the object pose
  std::string mReferenceFrameId;

  /// To retrieve resources from disk and from packages
  dart::common::ResourceRetrieverPtr mResourceRetriever;

  /// The reference frame of HERB to transform with respect to
  dart::dynamics::Frame* mReferenceLink;

  /// The pointer to the loader of configuration data
  std::shared_ptr<AprilTagsDatabase> mConfigData;

  /// For the ROS node that will work with the April Tags module
  ros::NodeHandle mNode;

  /// Listens to the transform attached to the node
  tf::TransformListener mListener;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_APRILTAGSMODULE_HPP_
