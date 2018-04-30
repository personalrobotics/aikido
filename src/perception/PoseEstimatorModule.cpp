#include "aikido/perception/PoseEstimatorModule.hpp"

#include <Eigen/Geometry>
#include <dart/utils/urdf/DartLoader.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "aikido/io/CatkinResourceRetriever.hpp"
#include "aikido/perception/shape_conversions.hpp"

namespace aikido {
namespace perception {

//=============================================================================
PoseEstimatorModule::PoseEstimatorModule(
    ros::NodeHandle nodeHandle,
    std::string markerTopic,
    std::shared_ptr<ObjectDatabase> configData,
    std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
    std::string referenceFrameId,
    dart::dynamics::Frame* referenceLink)
  : mNodeHandle(std::move(nodeHandle))
  , mMarkerTopic(std::move(markerTopic))
  , mConfigData(std::move(configData))
  , mResourceRetriever(std::move(resourceRetriever))
  , mReferenceFrameId(std::move(referenceFrameId))
  , mReferenceLink(std::move(referenceLink))
  , mTfListener(mNodeHandle)
{
  // Do nothing
}

//=============================================================================
bool PoseEstimatorModule::detectObjects(
    const aikido::planner::WorldPtr& env,
    ros::Duration timeout,
    ros::Time timestamp)
{
  // Checks detected objects, looks up the database,
  // and adds new skeletons to the world env
  bool any_detected = false;
  visualization_msgs::MarkerArrayConstPtr marker_message
      = ros::topic::waitForMessage<visualization_msgs::MarkerArray>(
          mMarkerTopic, mNodeHandle, timeout);

  // Making sure the Message from a pose estimator is non empty
  if (marker_message == nullptr)
  {
    dtwarn << "[PoseEstimatorModule::detectObjects] nullptr Marker Message "
           << mMarkerTopic << std::endl;
    return false;
  }

  if (marker_message->markers.empty())
  {
    dtwarn << "[PoseEstimatorModule::detectObjects] No markers on topic "
           << mMarkerTopic << std::endl;
    return false;
  }

  dart::utils::DartLoader urdf_loader;

  for (const auto& marker_transform : marker_message->markers)
  {
    const auto& marker_stamp = marker_transform.header.stamp;
    const auto& obj_key = marker_transform.text;
    const auto& obj_id = marker_transform.ns;
    const auto& detection_frame = marker_transform.header.frame_id;
    if (!timestamp.isValid() || marker_stamp < timestamp)
    {
      continue;
    }

    // If obj_key is "None",
    // remove a skeleton with obj_id from env
    // and move to next marker
    dart::dynamics::SkeletonPtr env_skeleton = env->getSkeleton(obj_id);
    if (!obj_key.compare("None"))
    {
      if (env_skeleton != nullptr)
      {
        env->removeSkeleton(env_skeleton);
      }
      continue;
    }

    std::string obj_name;
    dart::common::Uri obj_resource;
    Eigen::Isometry3d obj_offset;
    ros::Time t0 = ros::Time(0);

    // get the object name, resource, and offset from database by objectKey
    mConfigData->getObjectByKey(obj_key, obj_name, obj_resource, obj_offset);

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
      dtwarn << "[PoseEstimatorModule::detectObjects] TF timestamp is "
                "out-of-date compared to marker timestamp "
             << ex.what() << std::endl;
      continue;
    }

    // Get orientation of marker
    Eigen::Isometry3d marker_pose
        = aikido::perception::convertROSPoseToEigen(marker_transform.pose);
    // Get the transform as a pose matrix
    Eigen::Isometry3d frame_pose
        = aikido::perception::convertStampedTransformToEigen(transform);
    // Compose to get actual skeleton pose
    Eigen::Isometry3d obj_pose = frame_pose * marker_pose * obj_offset;

    Eigen::Isometry3d link_offset = mReferenceLink->getWorldTransform();
    obj_pose = link_offset * obj_pose;

    bool is_new_obj;
    dart::dynamics::SkeletonPtr obj_skeleton;

    // Check if skel in World
    // If there is, update its pose
    // If not, add skeleton to env
    // A pose estimator module should provide the unique obj_id per object
    if (env_skeleton == nullptr)
    {
      is_new_obj = true;
      obj_skeleton
          = urdf_loader.parseSkeleton(obj_resource, mResourceRetriever);

      if (!obj_skeleton)
      {
        dtwarn
            << "[PoseEstimatorModule::detectObjects] Failed to load skeleton "
            << "for URI " << obj_resource.toString() << std::endl;
        continue;
      }
      obj_skeleton->setName(obj_id);
    }
    else
    {
      is_new_obj = false;
      obj_skeleton = env_skeleton;
    }

    dart::dynamics::Joint* jtptr;
    if (obj_skeleton->getNumDofs() > 0)
    {
      jtptr = obj_skeleton->getJoint(0);
    }
    else
    {
      dtwarn << "[PoseEstimatorModule::detectObjects] Skeleton " << obj_name
             << " has 0 DOFs! \n";
      continue;
    }

    // Downcast Joint to FreeJoint
    dart::dynamics::FreeJoint* freejtptr
        = dynamic_cast<dart::dynamics::FreeJoint*>(jtptr);

    if (freejtptr == nullptr)
    {
      dtwarn << "[PoseEstimatorModule::detectObjects] Could not cast the joint "
                "of the body to a Free Joint so ignoring the object "
             << obj_name << std::endl;
      continue;
    }

    freejtptr->setTransform(obj_pose);

    if (is_new_obj)
    {
      env->addSkeleton(obj_skeleton);
    }

    any_detected = true;
  }

  if (!any_detected)
  {
    dtwarn << "[PoseEstimatorModule::detectObjects] No marker up-to-date with "
              "timestamp parameter"
           << std::endl;
    return false;
  }

  return true;
}

} // namespace perception
} // namespace aikido
