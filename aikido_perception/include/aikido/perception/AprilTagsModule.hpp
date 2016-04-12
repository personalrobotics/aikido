#ifndef AIKIDO_PERCEPTION_APRILTAGSMODULE_H
#define AIKIDO_PERCEPTION_APRILTAGSMODULE_H

#include <string>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <aikido/perception/ConfigDataLoader.hpp>
#include <tf/transform_listener.h>
#include "yaml-cpp/yaml.h"

#include <dart/dart.h>
#include <aikido/util/CatkinResourceRetriever.hpp>

#include "PerceptionModule.hpp"


namespace aikido{
namespace perception{

class AprilTagsModule : public virtual PerceptionModule
{
public:
	AprilTagsModule(const ros::NodeHandle node, const std::string markerTopic, ConfigDataLoader* configData,
					const dart::common::ResourceRetrieverPtr& resourceRetriever, const std::string urdfPath, 
					const std::string destinationFrame, dart::dynamics::Frame* referenceLink);
	virtual ~AprilTagsModule() = default;


	void detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,double _timeout=10.0, ros::Time timestamp=ros::Time::now()) override; 


private:
	//Member variables
	std::string mMarkerTopic;
	std::string mUrdfPath;
	std::string mDestinationFrame;
	dart::common::ResourceRetrieverPtr mResourceRetrieverPtr;
	dart::dynamics::Frame* mReferenceLink;

	ConfigDataLoader *mConfigData;
	tf::TransformListener mListener;

	ros::NodeHandle mNode;
	YAML::Node mTagData;

};

} //namespace perception
} //namespace aikido

#endif //AIKIDO_PERCEPTION_APRILTAGSMODULE_H