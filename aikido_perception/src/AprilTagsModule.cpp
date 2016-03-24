#include <aikido/perception/AprilTagsModule.hpp>
#include <aikido/perception/yaml_conversion.hpp>
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

AprilTagsModule::AprilTagsModule(ros::NodeHandle _node,const std::string _marker_topic, const std::string _marker_data_path,
								const dart::common::ResourceRetrieverPtr& _delegate, const std::string _urdf_path,	
								const std::string _destination_frame, dart::dynamics::BodyNode* _reference_link):
		node_(_node),
		marker_topic(_marker_topic),
		marker_data_path(_marker_data_path),
		delegate(_delegate),
		urdf_path(_urdf_path),
		destination_frame(_destination_frame),
		reference_link(_reference_link)
{
	//Load JSON file
	tag_data = YAML::LoadFile(marker_data_path.c_str());
}


void AprilTagsModule::detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,double timeout)
{
	//Looks at all detected tags, looks up config file 
	//Appends new skeletons to skeleton list
	visualization_msgs::MarkerArrayConstPtr marker_message
 			= ros::topic::waitForMessage<visualization_msgs::MarkerArray>(marker_topic,node_,ros::Duration(timeout));

	for(size_t i=0; i < marker_message->markers.size(); i++)
	{
		//Get marker, tag ID, and detection transform name 
		visualization_msgs::Marker marker_transform = marker_message->markers[i];
		std::string tag_name = marker_transform.ns;
		std::string detection_frame(marker_transform.header.frame_id);

		//check if marker namespace in tag_file
		if (tag_data[tag_name]){

			//Get parsed tag file and offset info!
			std::string body_name;
			Eigen::Matrix4d skel_offset;
			getTagNameOffset(tag_name,body_name,skel_offset);

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
				listener.waitForTransform(destination_frame,detection_frame,
					ros::Time(0),ros::Duration(timeout));

				listener.lookupTransform(destination_frame,detection_frame,
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
			Eigen::Isometry3d link_offset = reference_link->getWorldTransform();
			skel_pose = link_offset * skel_pose;

			//Check if skel in skel_list
			//If there then update pose
			//If not then append skeleton
			std::string skel_name = body_name.substr(0,body_name.find('.'));
			skel_name.append(std::to_string(marker_transform.id));
			bool is_new_skel = true; 

			for(size_t j=0;j < skeleton_list.size(); j++)
			{
				dart::dynamics::SkeletonPtr this_skel = skeleton_list[j];
				if(this_skel->getName() == skel_name){
					//Exists - just update pose
					is_new_skel = false;
					//Assumes single joint body
					dart::dynamics::Joint* jtptr = this_skel->getJoint(0);

					//Downcast Joint to FreeJoint
					dart::dynamics::FreeJoint* freejtptr = dynamic_cast<dart::dynamics::FreeJoint*>(jtptr);

					//Check if successful down-cast
					if(freejtptr == NULL){
						throw std::bad_cast();
					}
					freejtptr->setTransform(skel_pose);
					break;
				}
			}

			if(is_new_skel){
				//Read Skeleton file corresp. to body_name
				//TODO  - append path correctly
				std::string body_path(urdf_path);
				body_path.append(body_name);
				const auto resourceRetriever = std::make_shared<aikido::util::CatkinResourceRetriever>();
				dart::utils::DartLoader urdfLoader;
				dart::dynamics::SkeletonPtr new_skel = 
					urdfLoader.parseSkeleton(body_path,delegate);
				new_skel->setName(skel_name);
				dart::dynamics::Joint* jtptr = new_skel->getJoint(0);
				dart::dynamics::FreeJoint* freejtptr = dynamic_cast<dart::dynamics::FreeJoint*>(jtptr);

				if(freejtptr == NULL){
					throw std::bad_cast();
				}
				freejtptr->setTransform(skel_pose);

				//Append to skeleton list
				skeleton_list.push_back(new_skel);
			}
		}

	}

}

void AprilTagsModule::getTagNameOffset(std::string tag_name, std::string& body_name, Eigen::Matrix4d& body_offset)
{
	//For NEWER JSON file
	//Assumes field in file
		
	YAML::Node name_offset = tag_data[tag_name];
	body_name = name_offset["name"].as<std::string>();
	body_offset = name_offset["offset"].as<Eigen::Matrix4d>();
}



} //namespace perception
} //namespace aikido