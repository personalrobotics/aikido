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
								  dart::common::ResourceRetrieverPtr& resourceRetriever,  std::string urdfPath,	
								  std::string referenceFrameId, dart::dynamics::Frame* referenceLink):
		mNode(node),
		mMarkerTopic(markerTopic),
		mConfigData(configData),
		mResourceRetrieverPtr(resourceRetriever),
		mUrdfPath(urdfPath),
		mReferenceFrameId(referenceFrameId),
		mReferenceLink(referenceLink)
{
}

//================================================================================================================================
void AprilTagsModule::detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,double _timeout, ros::Time timestamp)
{
	//Looks at all detected tags, looks up config file 
	//Appends new skeletons to skeleton list
	visualization_msgs::MarkerArrayConstPtr marker_message
 			= ros::topic::waitForMessage<visualization_msgs::MarkerArray>(mMarkerTopic,mNode,ros::Duration(_timeout));

	for(size_t i=0; i < marker_message->markers.size(); i++)
	{
		//Get marker, tag ID, and detection transform name 
		visualization_msgs::Marker marker_transform = marker_message->markers[i];
		ros::Time marker_stamp = marker_transform.header.stamp;
		std::string tag_name = marker_transform.ns;
		std::string detection_frame(marker_transform.header.frame_id);

		if(marker_stamp < timestamp){
			dtwarn << "Marker stamp " << marker_stamp<< " for " << tag_name 
			<< "is behind timestamp parameter: " <<timestamp <<"\n";

			continue;
		}

		std::string body_name;
		Eigen::Isometry3d skel_offset;

		//check if tag name is in database
		if (mConfigData->getTagNameOffset(tag_name,body_name,skel_offset))
		{

			//Get orientation of marker
			/*
			Eigen::Quaterniond marker_orien(marker_transform.pose.orientation.w,
									marker_transform.pose.orientation.x,
									marker_transform.pose.orientation.y,
									marker_transform.pose.orientation.z);
			Eigen::Vector3d marker_pos(marker_transform.pose.position.x,marker_transform.pose.position.y,marker_transform.pose.position.z);

			//Get marker_pose from above data
			Eigen::Isometry3d marker_pose = Eigen::Isometry3d::Identity();
			marker_pose.rotate(marker_orien);
			marker_pose.translation() = marker_pos;
			*/
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
			catch(tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				throw std::runtime_error("No transform!");
			}

			//Get frame pose
			/*
			tf::Quaternion tf_quat(transform.getRotation());
			Eigen::Quaterniond frame_quat(tf_quat.getW(),tf_quat.getX(),tf_quat.getY(),tf_quat.getZ());
			Eigen::Vector3d frame_trans(transform.getOrigin());
			
			Eigen::Isometry3d frame_pose = Eigen::Isometry3d::Identity();
			frame_pose.rotate(frame_quat);
			frame_pose.translation() = frame_trans;
			*/
			Eigen::Isometry3d frame_pose = aikido::perception::convertStampedTransformToEigen(transform);

			//Compose to get actual skeleton pose
			Eigen::Isometry3d temp_pose = frame_pose * marker_pose;
			Eigen::Isometry3d skel_pose = temp_pose * skel_offset;

			Eigen::Isometry3d link_offset = mReferenceLink->getWorldTransform();
			skel_pose = link_offset * skel_pose;

			//Check if skel in skel_list
			//If there then update pose
			//If not then append skeleton
			std::string skel_name = body_name.substr(0,body_name.find('.'));
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
				std::string body_path(mUrdfPath);
				body_path.append(body_name);
				dart::utils::DartLoader urdfLoader;
				
				skel_to_update = 
					urdfLoader.parseSkeleton(body_path,mResourceRetrieverPtr);
				
				if(!skel_to_update)
					std::cout<<"Empty skel ptr!"<<std::endl;
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

}


} //namespace perception
} //namespace aikido