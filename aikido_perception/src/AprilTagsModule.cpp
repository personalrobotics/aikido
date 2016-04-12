#include <aikido/perception/AprilTagsModule.hpp>
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

AprilTagsModule::AprilTagsModule(const ros::NodeHandle node,const std::string markerTopic, ConfigDataLoader* configData,
								const dart::common::ResourceRetrieverPtr& resourceRetriever, const std::string urdfPath,	
								const std::string destinationFrame, dart::dynamics::Frame* referenceLink):
		mNode(node),
		mMarkerTopic(markerTopic),
		mConfigData(configData),
		mResourceRetrieverPtr(resourceRetriever),
		mUrdfPath(urdfPath),
		mDestinationFrame(destinationFrame),
		mReferenceLink(referenceLink)
{
}


void AprilTagsModule::detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,double _timeout, ros::Time timestamp)
{
	//Looks at all detected tags, looks up config file 
	//Appends new skeletons to skeleton list
	visualization_msgs::MarkerArrayConstPtr marker_message
 			= ros::topic::waitForMessage<visualization_msgs::MarkerArray>(mMarkerTopic,mNode,ros::Duration(_timeout));

	for(size_t i=0; i < marker_message->markers.size(); i++)
	{
		std::cout<<"Iteration "<<i<<std::endl;
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
		Eigen::Matrix4d skel_offset;

		//check if tag name is in database
		if (mConfigData->getTagNameOffset(tag_name,body_name,skel_offset))
		{
			//Get orientation of marker
			Eigen::Quaterniond orien(marker_transform.pose.orientation.w,
									marker_transform.pose.orientation.x,
									marker_transform.pose.orientation.y,
									marker_transform.pose.orientation.z);

			//Get in 4x4 pose form
			Eigen::Vector3d qpos(marker_transform.pose.position.x,marker_transform.pose.position.y,marker_transform.pose.position.z);
			Eigen::Matrix3d qrot(orien.toRotationMatrix());
			Eigen::Matrix4d marker_pose;
			marker_pose<< qrot,qpos,0,0,0,1;

			//For the frame-frame transform
			tf::StampedTransform transform;

			try{
				mListener.waitForTransform(mDestinationFrame,detection_frame,
					ros::Time(0),ros::Duration(_timeout));

				mListener.lookupTransform(mDestinationFrame,detection_frame,
					ros::Time(0), transform);
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
			}

			//Get frame pose
			tf::Quaternion tf_quat(transform.getRotation());

			Eigen::Quaterniond frame_quat(tf_quat.getW(),tf_quat.getX(),tf_quat.getY(),tf_quat.getZ());
			Eigen::Vector3d frame_trans(transform.getOrigin());
			Eigen::Matrix3d frame_rot(frame_quat.toRotationMatrix());
			Eigen::Matrix4d frame_pose;
			frame_pose<<frame_rot,frame_trans,0,0,0,1;

			//Compose to get actual skeleton pose
			Eigen::Isometry3d temp_pose;
			temp_pose.matrix() = frame_pose * marker_pose;
			Eigen::Isometry3d skel_pose;
			skel_pose.matrix() = temp_pose * skel_offset;
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

				std::cout<<"Body path is "<<body_path<<std::endl;
				
				skel_to_update = 
					urdfLoader.parseSkeleton(body_path,mResourceRetrieverPtr);
				
				std::cout<<"Skeleton loaded"<<std::endl;
				if(!skel_to_update)
					std::cout<<"Empty skel ptr!"<<std::endl;
				skel_to_update->setName(skel_name);

			}
			else{
				//Get existing skeletonPtr
				is_new_skel = false;
				skel_to_update = *it;
			}

			std::cout<<"Checking skel joints"<<std::endl;
			//Common stuff for old or new nodes
			//Ignore if body has > 1 joint - NEED TO GENERALIZE
			if(skel_to_update->getNumJoints() != 1){
				dtwarn << "Ignoring skeleton " << skel_to_update->getName()
					<< "because it does not have 1 joint \n";
				continue;
			}

			dart::dynamics::Joint* jtptr = skel_to_update->getJoint(0);

			//Downcast Joint to FreeJoint
			dart::dynamics::FreeJoint* freejtptr = dynamic_cast<dart::dynamics::FreeJoint*>(jtptr);

			//Check if successful down-cast
			if(freejtptr == NULL){
				std::cerr << "Could not cast the joint of the body to a Free Joint " << std::endl;
				throw std::bad_cast();
			}
			std::cout<<"Skel pose";
			freejtptr->setTransform(skel_pose);

			if(is_new_skel){
				//Append to list
				std::cout<<"Append to list"<<std::endl;
				skeleton_list.push_back(skel_to_update);
			}

		}

	}

}


} //namespace perception
} //namespace aikido