/** 
 * @file AprilTagsModule.cpp
 * @author Shushman Choudhury
 * @date Apr 14, 2016
 * @brief The implementation of the April Tags detector. The function detectObjects
 * waits for the next ROS message on the specified topic.
 */

#include <aikido/perception/AprilTagsModule.hpp>
#include <aikido/perception/shape_conversions.hpp>
#include "dart/common/Console.h"
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

namespace aikido{
namespace perception{

//================================================================================================================================

AprilTagsModule::AprilTagsModule( ros::NodeHandle node, std::string markerTopic, std::shared_ptr<ConfigDataLoader> configData,
								  const dart::common::ResourceRetrieverPtr& resourceRetriever,	
								  std::string referenceFrameId, dart::dynamics::Frame* referenceLink):
		mNode(node),
		mMarkerTopic(markerTopic),
		mConfigData(configData),
		mResourceRetriever(resourceRetriever),
		mReferenceFrameId(referenceFrameId),
		mReferenceLink(referenceLink)
{
}

//================================================================================================================================
void AprilTagsModule::detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,double _timeout, ros::Time timestamp)
{
	bool transform_lookedup = false;
	//Looks at all detected tags, looks up config file 
	//Appends new skeletons to skeleton list
	do{
		visualization_msgs::MarkerArrayConstPtr marker_message
	 			= ros::topic::waitForMessage<visualization_msgs::MarkerArray>(mMarkerTopic,mNode,ros::Duration(_timeout));

		for (const auto& marker_transform : marker_message->markers)
		{
			//Get marker, tag ID, and detection transform name 
			const auto& marker_stamp = marker_transform.header.stamp;
			const auto& tag_name = marker_transform.ns;
			const auto& detection_frame = marker_transform.header.frame_id;

			if(marker_stamp < timestamp){
				dtwarn << "Marker stamp " << marker_stamp<< " for " << tag_name 
				<< "is behind timestamp parameter: " <<timestamp <<"\n";

				continue;
			}

			std::string skel_name;
			Eigen::Isometry3d skel_offset;
			dart::common::Uri skel_resource;

			//check if tag name is in database
			if (mConfigData->getTagNameOffset(tag_name,skel_name,skel_resource,skel_offset))
			{

				//Get orientation of marker
				Eigen::Isometry3d marker_pose = aikido::perception::convertROSPoseToEigen(marker_transform.pose);


				//For the frame-frame transform
				tf::StampedTransform transform;
				tf::TransformListener listener;

				try{
					listener.waitForTransform(mReferenceFrameId,detection_frame,
						ros::Time(0),ros::Duration(_timeout));

					listener.lookupTransform(mReferenceFrameId,detection_frame,
						ros::Time(0), transform);
				}
				catch(const tf::ExtrapolationException& ex){
					ROS_ERROR("%s",ex.what());
					continue;
				}
				catch(const tf::TransformException& ex){
					ROS_ERROR("%s",ex.what());
					throw std::runtime_error("TF Error which cannot be recovered");
				}

				transform_lookedup = true;

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
				skel_name.append(std::to_string(marker_transform.id));
				bool is_new_skel;

				//Search skeleton_list for skeleton
				const auto it = std::find_if(std::begin(skeleton_list), std::end(skeleton_list),
					[&](const dart::dynamics::SkeletonPtr& skeleton)
				  	{
						return skeleton->getName() == skel_name;
				  	}
				);

				dart::dynamics::SkeletonPtr skel_to_update;

				if (it == std::end(skeleton_list)){
					//New skeleton
					is_new_skel = true;
					dart::utils::DartLoader urdfLoader;
					skel_to_update = 
						urdfLoader.parseSkeleton(skel_resource,mResourceRetriever);
					
					if(!skel_to_update)
						dtwarn<<"Empty SkeletonPtr for "<<skel_name<<std::endl;
					skel_to_update->setName(skel_name);

				}
				else{
					//Get existing skeletonPtr
					is_new_skel = false;
					skel_to_update = *it;
				}


				//Get root joint of skeleton
				dart::dynamics::Joint* jtptr = skel_to_update->getJoint(0);

				//Downcast Joint to FreeJoint
				dart::dynamics::FreeJoint* freejtptr = dynamic_cast<dart::dynamics::FreeJoint*>(jtptr);

				//Check if successful down-cast
				if(freejtptr == NULL){
					dtwarn << "Could not cast the joint of the body to a Free Joint so ignoring marker "<<tag_name << std::endl;
					continue;
				}

				freejtptr->setTransform(skel_pose);

				if(is_new_skel){
					//Append to list
					skeleton_list.push_back(skel_to_update);
				}

			}

		}
	}while(!transform_lookedup);
}


} //namespace perception
} //namespace aikido