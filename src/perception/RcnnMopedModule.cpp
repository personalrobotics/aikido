#include <aikido/perception/RcnnMopedModule.hpp>

#include <Eigen/Geometry>
#include <dart/utils/urdf/DartLoader.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <aikido/perception/shape_conversions.hpp>

namespace aikido {
namespace perception {

RcnnMopedModule::RcnnMopedModule(
    ros::NodeHandle nodeHandle,
    std::string markerTopic,
    dart::common::ResourceRetrieverPtr resourceRetriever,
    std::string referenceFrameId,
    dart::dynamics::Frame* referenceLink)
  : mNodeHandle(std::move(nodeHandle))
  , mMarkerTopic(std::move(markerTopic))
  , mReferenceFrameId(std::move(referenceFrameId))
  , mResourceRetriever(std::move(resourceRetriever))
  , mReferenceLink(std::move(referenceLink))
  , mTfListener(mNodeHandle)
{
  // Do nothing
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
    const auto& tag_name = marker_transform.ns;
    const auto& detection_frame = marker_transform.header.frame_id;
    if (!timestamp.isValid() || marker_stamp < timestamp)
    {
      continue;
    }

    any_valid = true;
    std::string skel_name;
    Eigen::Isometry3d skel_offset;
    dart::common::Uri skel_resource;
    ros::Time t0 = ros::Time(0);
   
    // Get orientation of marker
    Eigen::Isometry3d marker_pose =
        aikido::perception::convertROSPoseToEigen(marker_transform.pose);

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

    // Get the transform as a pose matrix
    Eigen::Isometry3d frame_pose =
        aikido::perception::convertStampedTransformToEigen(transform);

    // Compose to get actual skeleton pose
    Eigen::Isometry3d temp_pose = frame_pose * marker_pose;
    Eigen::Isometry3d skel_pose = temp_pose * skel_offset;
    Eigen::Isometry3d link_offset = mReferenceLink->getWorldTransform();
    skel_pose = link_offset * skel_pose;

    // Check if skel in skel_list
    skel_name.append(std::to_string(marker_transform.id));
    bool is_new_skel;

    // Search the environment for skeleton
    dart::dynamics::SkeletonPtr skel_to_update;

    dart::dynamics::SkeletonPtr env_skeleton = env->getSkeleton(skel_name);

    if (env_skeleton == nullptr)
    {
      // Getting the model for the new object
      is_new_skel = true;
      dart::utils::DartLoader urdfLoader;
      skel_to_update =
          urdfLoader.parseSkeleton(skel_resource, mResourceRetriever);

      if (!skel_to_update)
      {
        dtwarn << "[RcnnMopedModule::detectObjects] Failed to load skeleton "
                  "for URI "
               << skel_resource.toString() << std::endl;
      }

      skel_to_update->setName(skel_name);
    }
    else
    {
      is_new_skel = false;
      skel_to_update = env_skeleton;
    }

    dart::dynamics::Joint* jtptr;
    // Get root joint of skeleton
    if (skel_to_update->getNumDofs() > 0)
    {
      jtptr = skel_to_update->getJoint(0);
    }
    else
    {
      dtwarn << "[RcnnMopedModule::detectObjects] Skeleton " << skel_name
             << " has 0 DOFs! " << std::endl;
      continue;
    }
    
    // Downcast Joint to FreeJoint
    dart::dynamics::FreeJoint* freejtptr =
        dynamic_cast<dart::dynamics::FreeJoint*>(jtptr);

    // Check if successful down-cast
    if (freejtptr == nullptr)
    {
      dtwarn << "[RcnnMopedModule::detectObjects] Could not cast the joint "
                "of the body to a Free Joint so ignoring marker "
             << tag_name << std::endl;
      continue;
    }

    freejtptr->setTransform(skel_pose);

    if (is_new_skel)
    {
      // Adding new skeleton to the world env
      env->addSkeleton(skel_to_update);
    }
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
