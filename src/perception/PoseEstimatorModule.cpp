#include "aikido/perception/PoseEstimatorModule.hpp"

#include <Eigen/Geometry>
#include <dart/utils/urdf/DartLoader.hpp>
#include <ros/topic.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "aikido/perception/shape_conversions.hpp"

namespace aikido {
namespace perception {

//=============================================================================
PoseEstimatorModule::PoseEstimatorModule(
    ros::NodeHandle nodeHandle,
    const std::string& markerTopic,
    std::shared_ptr<AssetDatabase> assetData,
    std::shared_ptr<aikido::io::CatkinResourceRetriever> resourceRetriever,
    const std::string& referenceFrameId,
    dart::dynamics::Frame* referenceLink)
  : mNodeHandle(std::move(nodeHandle))
  , mMarkerTopic(std::move(markerTopic))
  , mAssetData(std::move(assetData))
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
    ros::Time timestamp,
    std::vector<DetectedObject>* detectedObjects)
{
  // Checks detected objects, looks up the database,
  // and adds new skeletons to the world env
  bool any_detected = false;
  visualization_msgs::MarkerArrayConstPtr markerMessage
      = ros::topic::waitForMessage<visualization_msgs::MarkerArray>(
          mMarkerTopic, mNodeHandle, timeout);

  // Making sure the Message from a pose estimator is non empty
  if (!markerMessage)
  {
    dtwarn << "[PoseEstimatorModule::detectObjects] nullptr Marker Message "
           << mMarkerTopic << std::endl;
    return false;
  }

  if (markerMessage->markers.empty())
  {
    dtwarn << "[PoseEstimatorModule::detectObjects] No markers on topic "
           << mMarkerTopic << std::endl;
    return false;
  }

  dart::utils::DartLoader urdfLoader;

  for (const auto& markerTransform : markerMessage->markers)
  {
    // TODO: Add DELETE_ALL Functionality
    // TODO: Update when we move over to ROS Kinetic. Indigo doesn't
    // have the enum value.
    // if (markerTransform.action == visualization_msgs::Marker::DELETEALL) {
    if (markerTransform.action == 3)
    {
      dtwarn << "[PoseEstimatorModule::detectObjects] We cannot currently "
                "handle DELETE_ALL markers."
             << std::endl;
      continue;
    }
    const auto& markerStamp = markerTransform.header.stamp;
    const auto& objectId = markerTransform.id;
    const auto& objectName = markerTransform.ns;
    const auto& detectionFrame = markerTransform.header.frame_id;
    if (!timestamp.isValid() || markerStamp < timestamp)
    {
      continue;
    }

    DetectedObject thisObject;
    try
    {

      // Initialize a DetectedObject class for this object
      thisObject = DetectedObject(
          objectName, objectId, detectionFrame, markerTransform.text);
    }
    catch (std::invalid_argument& e)
    {
      dtwarn << e.what();
      continue;
    }

    const std::string objUid = thisObject.getUid();

    // If markerTransform.action is "DELETE",
    // remove a skeleton with objUid from env
    dart::dynamics::SkeletonPtr env_skeleton = env->getSkeleton(objUid);
    if (markerTransform.action == visualization_msgs::Marker::DELETE)
    {
      if (env_skeleton != nullptr)
      {
        env->removeSkeleton(env_skeleton);
      }
      continue;
    }

    const std::string assetKey = thisObject.getAssetKey();
    if (assetKey.empty())
    {
      dtwarn << "[PoseEstimatorModule::detectObjects] Invalid YAML String in "
                "Marker: "
             << objUid << std::endl;
      continue;
    }

    dart::common::Uri objResource;
    Eigen::Isometry3d objOffset;
    ros::Time t0 = ros::Time(0);

    // Get the object name, resource, and offset from database by assetKey
    try
    {
      mAssetData->getAssetByKey(assetKey, objResource, objOffset);
    }
    catch (std::runtime_error& e)
    {
      dtwarn << e.what() << std::endl;
      continue;
    }

    tf::StampedTransform transform;
    try
    {
      mTfListener.waitForTransform(
          mReferenceFrameId, detectionFrame, t0, timeout);

      mTfListener.lookupTransform(
          mReferenceFrameId, detectionFrame, t0, transform);
    }
    catch (const tf::ExtrapolationException& ex)
    {
      dtwarn << "[PoseEstimatorModule::detectObjects] TF timestamp is "
                "out-of-date compared to marker timestamp "
             << ex.what() << std::endl;
      continue;
    }

    // Get orientation of marker
    Eigen::Isometry3d markerPose
        = aikido::perception::convertROSPoseToEigen(markerTransform.pose);
    // Get the transform as a pose matrix
    Eigen::Isometry3d framePose
        = aikido::perception::convertStampedTransformToEigen(transform);
    // Compose to get actual skeleton pose
    Eigen::Isometry3d objPose = framePose * markerPose * objOffset;

    Eigen::Isometry3d linkOffset = mReferenceLink->getWorldTransform();
    objPose = linkOffset * objPose;

    bool isNewObj;
    dart::dynamics::SkeletonPtr objSkeleton;

    // Check if skel in World
    // If there is, update its pose
    // If not, add skeleton to env
    if (!env_skeleton)
    {
      isNewObj = true;
      objSkeleton = urdfLoader.parseSkeleton(objResource, mResourceRetriever);

      if (!objSkeleton)
      {
        dtwarn
            << "[PoseEstimatorModule::detectObjects] Failed to load skeleton "
            << "for URI " << objResource.toString() << std::endl;
        continue;
      }
      objSkeleton->setName(objUid);
    }
    else
    {
      isNewObj = false;
      objSkeleton = env_skeleton;
    }

    thisObject.setMetaSkeleton(objSkeleton);

    dart::dynamics::Joint* jtptr;
    if (objSkeleton->getNumDofs() > 0)
    {
      jtptr = objSkeleton->getJoint(0);
    }
    else
    {
      dtwarn << "[PoseEstimatorModule::detectObjects] Skeleton " << objectName
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
             << objectName << std::endl;
      continue;
    }

    freejtptr->setTransform(objPose);

    if (isNewObj)
    {
      env->addSkeleton(objSkeleton);
    }

    // Add object to output vector, if available
    if (detectedObjects)
    {
      detectedObjects->push_back(thisObject);
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
