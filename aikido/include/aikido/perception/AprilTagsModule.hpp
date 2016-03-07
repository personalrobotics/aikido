#ifndef AIKIDO_PERCEPTION_APRILTAGSMODULE_H
#define AIKIDO_PERCEPTION_APRILTAGSMODULE_H

#include <string>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <tf/transform_listener.h>
#include "yaml-cpp/yaml.h"

#include <dart/dart.h>
#include <aikido/util/CatkinResourceRetriever.hpp>


namespace aikido{
namespace perception{

class AprilTagsModule : public PerceptionModule
{
public:
	AprilTagsModule(ros::NodeHandlePtr _node, std::string _marker_topic, std::string _marker_data_path,
					std::string _urdf_path, std::string _detection_frame,
					std::string _destination_frame, dart::dynamics::BodyNode* _reference_link);
	virtual ~AprilTagsModule()
	{
	}

	virtual void DetectObject();
	virtual void DetectObjects();


private:
	//Member variables
	std::string marker_topic;
	std::string marker_data_path;
	std::string urdf_path;
	std::string detection_frame;
	std::string destination_frame;
	dart::dynamics::BodyNode* reference_link;

	tf::TransformListener listener;

	ros::NodeHandlePtr node_;
	YAML::Node tag_data;

	void Update(double timeout=10.0,vector<dart::dynamics::Skeleton> skel_list); //Update BodyNodes after re-detection
	void GetTagNameOffset(std::string tag_name, std::string& body_name, Eigen::Matrix4d& body_offset);

};

} //namespace perception
} //namespace aikido

#endif AIKIDO_PERCEPTION_APRILTAGSMODULE_H