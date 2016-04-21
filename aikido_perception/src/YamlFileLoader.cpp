/** 
 * @file YamlFileLoader.cpp
 * @author Shushman Choudhury
 * @date Apr 20, 2016
 * @brief The instance of the configuration data loader for marker-based perception which
 * reads a YAML file.
 */


#include <dart/common/Console.h>
#include <dart/common/LocalResourceRetriever.h>
#include <aikido/perception/YamlFileLoader.hpp>

namespace aikido{
namespace perception{

//================================================================================================================================
YamlFileLoader::YamlFileLoader(const dart::common::ResourceRetrieverPtr& resourceRetriever,
                               dart::common::Uri configDataURI)
{
    
    dart::common::ResourceRetrieverPtr _resourceRetrieverPtr = resourceRetriever;

    //Read JSON file into string
    if(!_resourceRetrieverPtr){
        dtwarn << "[YamlFileLoader] ResourceRetrieverPtr given is null! "<<std::endl;
        throw std::runtime_error("Null pointer passed");
    }
    
    const dart::common::ResourcePtr resource = resourceRetriever->retrieve(configDataURI);
    if(!resource)
    {
        throw std::runtime_error("Failed opening URI.");
    }

    //Put file in string
    const size_t size = resource->getSize();
    std::string content;
    content.resize(size);
    if(resource->read(&content.front(), size, 1) != 1)
    {
        throw std::runtime_error("Failed reading from URI.");
    }

    //Load from string
    mTagData = YAML::Load(content);

}

//================================================================================================================================
bool YamlFileLoader::getTagNameOffset(const std::string& _tagName, std::string& body_name, dart::common::Uri& body_resource, Eigen::Isometry3d& body_offset)
{
    //Get name of object and pose for a given tag ID
    YAML::Node name_offset = mTagData[_tagName];
    if(name_offset){
        body_resource.fromString(name_offset["resource"].as<std::string>());
        body_name = name_offset["name"].as<std::string>();
        body_offset = name_offset["offset"].as<Eigen::Isometry3d>();
        return true;
    }
    else{
        dtwarn << "[YamlFileLoader] Could not find Tag ID " << _tagName <<"\n";
        return false;
    }
}


} //namespace perception

} //namespace aikido