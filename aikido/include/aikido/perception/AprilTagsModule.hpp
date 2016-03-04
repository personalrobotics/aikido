#ifndef AIKIDO_PERCEPTION_APRILTAGSMODULE_H
#define AIKIDO_PERCEPTION_APRILTAGSMODULE_H

#include <string>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "rapidjson/document.h"
#include "rapidjson/reader.h"
#include "rapidjson/stringbuffer.h"
#include <dart/dart.h>
#include <aikido/util/CatkinResourceRetriever.hpp>


namespace aikido{
namespace perception{

class AprilTagsModule : public PerceptionModule
{
public:
	AprilTagsModule(std::string marker_topic, std::string marker_data_path,
					std::string urdf_path, std::string detection_frame,
					std::string destination_frame, dart::dynamics::BodyNode* reference_link);
	virtual ~AprilTagsModule()
	{
	}

	virtual void DetectObject();
	virtual void DetectObjects();


protected:
	void ReloadKinbodyData();
	void Update(double timeout=10.0);
};

} //namespace perception
} //namespace aikido