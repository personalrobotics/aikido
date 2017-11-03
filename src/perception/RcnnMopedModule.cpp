#include <aikido/perception/RcnnMopedModule.hpp>

#include <Eigen/Geometry>
#include <dart/utils/urdf/DartLoader.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <aikido/perception/shape_conversions.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>

namespace aikido {
namespace perception {

RcnnMopedModule::RcnnMopedModule(
    ros::NodeHandle nodeHandle,
    std::string markerTopic,
    dart::common::ResourceRetrieverPtr resourceRetriever,
    std::string canUri,
    std::string referenceFrameId,
    dart::dynamics::Frame* referenceLink)
  : mNodeHandle(std::move(nodeHandle))
  , mMarkerTopic(std::move(markerTopic))
  , mReferenceFrameId(std::move(referenceFrameId))
  , mResourceRetriever(std::move(resourceRetriever))
  , mCanUri(std::move(canUri))
  , mReferenceLink(std::move(referenceLink))
  , mTfListener(mNodeHandle)
{
  // Do nothing
}


//=============================================================================
const dart::dynamics::SkeletonPtr makeBodyFromURDF(const std::string& uri,
    const Eigen::Isometry3d& transform)
{
  // Resolves package:// URIs by emulating the behavior of 'catkin_find'.
  const auto resourceRetriever =
      std::make_shared<aikido::io::CatkinResourceRetriever>();

  dart::utils::DartLoader urdfLoader;
  const dart::dynamics::SkeletonPtr skeleton = urdfLoader.parseSkeleton(
      uri, resourceRetriever);

  if (!skeleton)
  {
    throw std::runtime_error("unable to load '" + uri + "'");
  }

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))->setTransform(transform);

  return skeleton;
}

//=============================================================================
bool RcnnMopedModule::detectObjects(
    const aikido::planner::WorldPtr& env,
    ros::Duration timeout,
    ros::Time timestamp)
{
  bool any_valid = false;
  
  tf::TransformListener listener(mNodeHandle);
  
  visualization_msgs::MarkerArrayConstPtr marker_message =
      ros::topic::waitForMessage<visualization_msgs::MarkerArray>(
          mMarkerTopic, mNodeHandle, timeout);

  // Making sure the Message from MopedLite is non empty
  if (marker_message == nullptr)
  {
    dtwarn << "[RcnnMopedModule::detectObjects] nullptr Marker Message "
           << mMarkerTopic << std::endl;
    return false;
  }

  if (marker_message->markers.size() == 0)
  {
    dtwarn << "[RcnnMopedModule::detectObjects] No markers on topic "
           << mMarkerTopic << std::endl;
    return false;
  }

  for (const auto& marker_transform : marker_message->markers)
  {
    // Get marker, tag ID, and detection transform name
    const auto& marker_stamp = marker_transform.header.stamp;
    const auto& detection_frame = marker_transform.header.frame_id;
    if (!timestamp.isValid() || marker_stamp < timestamp)
    {
      continue;
    }

    ros::Time t0 = ros::Time(0);
   
    tf::StampedTransform transform;
    try
    {
      mTfListener.waitForTransform(
          mReferenceFrameId, detection_frame, t0, timeout);

      mTfListener.lookupTransform(
          mReferenceFrameId, detection_frame, t0, transform);
    }
    catch (const tf::ExtrapolationException& ex)
    {
      dtwarn << "[RcnnMopedModule::detectObjects] TF timestamp is "
                "out-of-date compared to marker timestamp "
             << ex.what() << std::endl;
      continue;
    }

    // Get orientation of marker
    Eigen::Isometry3d marker_pose =
        aikido::perception::convertROSPoseToEigen(marker_transform.pose);
    // Get the transform as a pose matrix
    Eigen::Isometry3d frame_pose =
        aikido::perception::convertStampedTransformToEigen(transform);
    // Compose to get actual skeleton pose
    Eigen::Isometry3d skel_pose = frame_pose * marker_pose;
    Eigen::Isometry3d link_offset = mReferenceLink->getWorldTransform();
    skel_pose = link_offset * skel_pose;

    dart::dynamics::SkeletonPtr canSkeletonPtr =
        makeBodyFromURDF(mCanUri, skel_pose);

    std::string skel_name = "can";
    skel_name.append(std::to_string(marker_transform.id));

    canSkeletonPtr->setName(skel_name);

    dart::dynamics::SkeletonPtr env_skeleton = env->getSkeleton(skel_name);

    if (env_skeleton != nullptr)
    {
      env->removeSkeleton(env_skeleton);
    }
    env->addSkeleton(canSkeletonPtr);

    any_valid = true;
  }

  if (!any_valid)
  {
    dtwarn << "[RcnnMopedModule::detectObjects] No marker up-to-date with "
              "timestamp parameter" << std::endl;
    return false;
  }

  return true;
}


}  // namespace perception
}  // namespace aikido
