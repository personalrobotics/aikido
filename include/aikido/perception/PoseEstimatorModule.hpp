#ifndef AIKIDO_PERCEPTION_POSEESTIMATORMODULE_HPP_
#define AIKIDO_PERCEPTION_POSEESTIMATORMODULE_HPP_

#include <string>
#include <dart/dart.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "aikido/io/CatkinResourceRetriever.hpp"
#include "aikido/io/yaml.hpp"
#include "aikido/perception/AssetDatabase.hpp"
#include "aikido/perception/DetectedObject.hpp"
#include "aikido/perception/PerceptionModule.hpp"

namespace aikido {
namespace perception {

/// Instantiates the \c PerceptionModule specifically for PoseEstimator.
///
/// It receives input from the PoseEstimator running in a separate process,
/// by subscribing to a \c visualization_msg::MarkerArray ROS topic
/// published by the PoseEstimator node.
/// It uses a \c AssetDatabase to resolve each marker to a
/// \c dart::common::Uri, and updates the environment which is an
/// \c aikido::planner::World.
class PoseEstimatorModule : public PerceptionModule
{
public:
  /// Construct a PoseEstimator receiver that subscribes to the specified topic
  /// where objects' pose information is being published as a
  /// \c visualization_msg::MarkerArray, uses a database loader for
  /// configuration information related to tags.
  ///
  ///	\param[in] nodeHandle The node handle to be passed to the detector
  ///	\param[in] markerTopic The name of the topic on which objects' pose
  /// information is being published
  /// \param[in] assetData The pointer to the loader of visual asset data (\see
  /// AssetDatabase)
  /// \param[in] resourceRetriever A CatkinResourceRetriever for resources
  /// related to config files and models
  ///	\param[in] referenceFrameId The desired reference frame for the
  /// object pose
  ///	\param[in] referenceLink A link on the robot with respect to which the
  /// pose is transformed
  PoseEstimatorModule(
      ros::NodeHandle nodeHandle,
      const std::string& markerTopic,
      std::shared_ptr<AssetDatabase> assetData,
      std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
      const std::string& referenceFrameId,
      dart::dynamics::Frame* referenceLink);

  virtual ~PoseEstimatorModule() = default;

  /// \copydoc PerceptionModule::detectObjects()
  /// Looks for the following information sent via ROS (see \c DetectedObject):
  /// visualization_msgs/Marker message like the following:
  /// Marker.header.frame_id -> detectionFrameID
  /// Marker.ns + "_" + Marker.id -> objUid (identity in planner::World)
  /// Marker.ns -> objAssetKey (determines visual asset, see AssetDatabase)
  bool detectObjects(
      const aikido::planner::WorldPtr& env,
      ros::Duration timeout = ros::Duration(),
      ros::Time timestamp = ros::Time(0.0),
      std::vector<DetectedObject>* detectedObjects = nullptr) override;

private:
  /// For the ROS node that will work with the April Tags module
  ros::NodeHandle mNodeHandle;

  /// The name of the ROS topic to read marker info from
  std::string mMarkerTopic;

  /// The pointer to the loader of visual asset data
  std::shared_ptr<AssetDatabase> mAssetData;

  /// To retrieve resources from disk and from packages
  std::shared_ptr<aikido::io::CatkinResourceRetriever> mResourceRetriever;

  /// The desired reference frame for the object pose
  std::string mReferenceFrameId;

  /// The reference frame of HERB to transform with respect to
  dart::dynamics::Frame* mReferenceLink;

  /// Listens to the transform attached to the node
  tf::TransformListener mTfListener;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_POSEESTIMATORMODULE_HPP_
