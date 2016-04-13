/** 
 * @file AprilTagsModule.hpp
 * @author Shushman Choudhury
 * @date Apr 14, 2016
 * @brief The header for the April Tags detector. The function detectObjects
 * waits for the next ROS message on the specified topic.
 */

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

/// The April Tag detector 
class AprilTagsModule : public virtual PerceptionModule
{
public:
	AprilTagsModule(const ros::NodeHandle node, const std::string markerTopic, ConfigDataLoader* configData,
					const dart::common::ResourceRetrieverPtr& resourceRetriever, const std::string urdfPath, 
					const std::string destinationFrame, dart::dynamics::Frame* referenceLink);
	virtual ~AprilTagsModule() = default;


	void detectObjects(std::vector<dart::dynamics::SkeletonPtr>& skeleton_list,double _timeout=10.0, ros::Time timestamp=ros::Time::now()) override; 


private:
	///The name of the ROS topic to read marker info from
	std::string mMarkerTopic;	

	///The path to the folder of URDF files
	std::string mUrdfPath;

	///The desired reference frame for the object pose	
	std::string mDestinationFrame;

	///To retrieve resources from disk and from packages
	dart::common::ResourceRetrieverPtr mResourceRetrieverPtr;
	
	///The reference frame of HERB to transform with respect to
	dart::dynamics::Frame* mReferenceLink;

	///The pointer to the loader of configuration data
	ConfigDataLoader *mConfigData;

	///For the ROS node that will work with the April Tags module
	ros::NodeHandle mNode;

};

} //namespace perception
} //namespace aikido

#endif //AIKIDO_PERCEPTION_APRILTAGSMODULE_H