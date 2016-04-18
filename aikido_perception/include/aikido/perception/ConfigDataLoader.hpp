#ifndef AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H
#define AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H

#include "yaml-cpp/yaml.h"
#include <aikido/perception/yaml_conversion.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <dart/common/Console.h>
#include <dart/common/LocalResourceRetriever.h>
#include <Eigen/Geometry>
#include <stdexcept>

namespace aikido{
namespace perception{

class ConfigDataLoader
{
public:
    ConfigDataLoader(const dart::common::ResourceRetrieverPtr& resourceRetriever,
                     const dart::common::Uri configDataURI):
        mResourceRetrieverPtr(resourceRetriever),
        mConfigDataURI(configDataURI)
        {}

    //Made  public so user can choose when to load config data
    void loadConfigData()
    {
        //Read JSON file into string
        if(!mResourceRetrieverPtr)
            mResourceRetrieverPtr = std::make_shared<dart::common::LocalResourceRetriever>();

        const dart::common::ResourcePtr resource = mResourceRetrieverPtr->retrieve(mConfigDataURI);
        if(!resource)
        {
            dtwarn<<"[ConfigDataLoader] Failed opening URI "<<mConfigDataURI.toString()<<"\n";
            throw std::runtime_error("Failed opening URI.");
        }

        //Put file in string
        const size_t size = resource->getSize();
        std::string content;
        content.resize(size);
        if(resource->read(&content.front(), size, 1) != 1)
        {
            dtwarn << "[ConfigDataLoader] Failed reading from URI '"
                << mConfigDataURI.toString() << "'.\n";
            throw std::runtime_error("Failed reading from URI.");
        }

        //Load from string
        mTagData = YAML::Load(content);

    }

    //Get name of object and pose for a given tag ID
    bool getTagNameOffset(const std::string _tagName, std::string& body_name, Eigen::Isometry3d& body_offset)
    {
        YAML::Node name_offset = mTagData[_tagName];
        if(name_offset){
            body_name = name_offset["name"].as<std::string>();
            Eigen::Matrix4d body_offset_mat = name_offset["offset"].as<Eigen::Matrix4d>();
            body_offset.matrix() = body_offset_mat;
            return true;
        }
        else{
            dtwarn << "[ConfigDataLoader] Could not find Tag ID " << _tagName <<"\n";
            return false;
        }
    }

private:

    //Member variables
    dart::common::ResourceRetrieverPtr mResourceRetrieverPtr;
    dart::common::Uri mConfigDataURI;
    YAML::Node mTagData;
    std::map<std::string,Eigen::Matrix4d> mConfigMap;
};



} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H