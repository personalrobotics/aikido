#include <aikido/perception/AprilTagsModule.hpp>
#include <aikido/perception/shape_conversions.hpp>
#include <dart/common/Console.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/topic.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/make_shared.hpp>
#include <Eigen/Geometry>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <stdexcept>
 #include <utility>

namespace aikido{
namespace perception{

//================================================================================================================================

AprilTagsModule::AprilTagsModule( ros::NodeHandle node, std::string markerTopic, std::shared_ptr<AprilTagsDatabase> configData,
								  const dart::common::ResourceRetrieverPtr resourceRetriever,	
								  std::string referenceFrameId, dart::dynamics::Frame* referenceLink):
		mNode(std::move(node)),
		mMarkerTopic(std::move(markerTopic)),
		mConfigData(std::move(configData)),
		mResourceRetriever(std::move(resourceRetriever)),
		mReferenceFrameId(std::move(referenceFrameId)),
		mReferenceLink(std::move(referenceLink)),
		mListener(mNode)
{
}

//================================================================================================================================
// bool AprilTagsModule::detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,ros::Duration timeout, ros::Time timestamp)
AprilTagsModule::detectObjects(const dart::simulation::WorldPtr env, ros::Duration timeout, ros::Time timestamp)
{
	bool any_valid = false;
	tf::TransformListener listener(mNode);
	//Looks at all detected tags, looks up config file 
	//Appends new skeletons to skeleton list
	visualization_msgs::MarkerArrayConstPtr marker_message
 			= ros::topic::waitForMessage<visualization_msgs::MarkerArray>(mMarkerTopic,mNode,timeout);

 	// Making sure the Apriltag Message is non empty
 	if(marker_message == NULL){
 		dtwarn<<"[AprilTagsModule::detectObjects] NULL Marker Message "<<mMarkerTopic<< std::endl;
 		return false;
 	}

 	if(marker_message->markers.size()==0){
 		dtwarn<<"[AprilTagsModule::detectObjects] No markers on topic "<<mMarkerTopic<<std::endl;
 		return false;
 	}

	for (const auto& marker_transform : marker_message->markers)
	{
		//Get marker, tag ID, and detection transform name 
		const auto& marker_stamp = marker_transform.header.stamp;
		const auto& tag_name = marker_transform.ns;
		const auto& detection_frame = marker_transform.header.frame_id;
		if(!timestamp.isValid() || marker_stamp < timestamp){
			continue;
		}

		any_valid = true;
		std::string skel_name;
		Eigen::Isometry3d skel_offset;
		dart::common::Uri skel_resource;

		//check if tag name is in database
		if (mConfigData->getTagNameOffset(tag_name,skel_name,skel_resource,skel_offset))
		{

			//Get orientation of marker
			Eigen::Isometry3d marker_pose = aikido::perception::convertROSPoseToEigen(marker_transform.pose);

			//For the frame-frame transform from TF
			// TODO: what happens if the TF transform is unavailable?
			tf::StampedTransform transform;

			try{
				mListener.waitForTransform(mReferenceFrameId,detection_frame,
					marker_stamp,timeout);

				mListener.lookupTransform(mReferenceFrameId,detection_frame,
					marker_stamp, transform);
			}
			catch(const tf::ExtrapolationException& ex){
				dtwarn<< "[AprilTagsModule::detectObjects] TF timestamp is out-of-date compared to marker timestamp " << ex.what();
				continue;
			}

			//Get the transform as a pose matrix
			Eigen::Isometry3d frame_pose = aikido::perception::convertStampedTransformToEigen(transform);

			//Compose to get actual skeleton pose
			Eigen::Isometry3d temp_pose = frame_pose * marker_pose;
			Eigen::Isometry3d skel_pose = temp_pose * skel_offset;

			Eigen::Isometry3d link_offset = mReferenceLink->getWorldTransform();
			skel_pose = link_offset * skel_pose;

			//Check if skel in skel_list
			//If there then update pose
			//If not then append skeleton
			//Resolving naming conflicts by adding the marker names.
			skel_name.append(std::to_string(marker_transform.id));
			bool is_new_skel;

			//Search skeleton_list for skeleton

			dart::dynamics::SkeletonPtr skel_to_update;
			dart::dynamics::SkeletonPtr env_skeleton = env->getSkeleton(skel_name)
			
			if(env_skeleton == NULL){
				is_new_skel = true;
				dart::utils::DartLoader urdfLoader;
				skel_to_update = 
					urdfLoader.parseSkeleton(skel_resource, mResourceRetriever);

					if(!skel_to_update)
						dtwarn<<"[AprilTagsModule::detectObjects] Failed to load skeleton for URI "<<skel_resource.toString()<<std::endl;
					skel_to_update->setName(skel_name);
			} else {
				is_new_skel = false;
				skel_to_update = env_skeleton
			}

			// const auto it = std::find_if(std::begin(skeleton_list), std::end(skeleton_list),
			// 	[&](const dart::dynamics::SkeletonPtr& skeleton)
			//   	{
			// 		return skeleton->getName() == skel_name;
			//   	}
			// );

			// dart::dynamics::SkeletonPtr skel_to_update;

			// if (it == std::end(skeleton_list)){
			// 	//New skeleton
			// 	is_new_skel = true;
			// 	dart::utils::DartLoader urdfLoader;
			// 	skel_to_update = 
			// 		urdfLoader.parseSkeleton(skel_resource,mResourceRetriever);
				
			// 	if(!skel_to_update)
			// 		dtwarn<<"[AprilTagsModule::detectObjects] Failed to load skeleton for URI "<<skel_resource.toString()<<std::endl;
			// 	skel_to_update->setName(skel_name);

			// }
			// else{
			// 	//Get existing skeletonPtr
			// 	is_new_skel = false;
			// 	skel_to_update = *it;
			// }


			dart::dynamics::Joint* jtptr;
			//Get root joint of skeleton
			if(skel_to_update->getNumDofs() > 0){
				jtptr = skel_to_update->getJoint(0);
			}
			else{
				dtwarn<< "[AprilTagsModule::detectObjects] Skeleton "<<skel_name<<" has 0 DOFs! \n";
				continue; 
			}

			//Downcast Joint to FreeJoint
			dart::dynamics::FreeJoint* freejtptr = dynamic_cast<dart::dynamics::FreeJoint*>(jtptr);

			//Check if successful down-cast
			if(freejtptr == nullptr){
				dtwarn << "[AprilTagsModule::detectObjects] Could not cast the joint of the body to a Free Joint so ignoring marker "<<tag_name << std::endl;
				continue;
			}

			freejtptr->setTransform(skel_pose);

			if(is_new_skel){
				//Append to list
				// skeleton_list.push_back(skel_to_update);
				env->addSkeleton(skel_to_update)
			}

		}

	}
	if(!any_valid){
		dtwarn<< "[AprilTagsModule::detectObjects] No marker up-to-date with timestamp parameter\n";
		return false;
	}

	return true;

}


} //namespace perception
} //namespace aikido
