#include <dart/common/Console.h>
#include <dart/common/LocalResourceRetriever.h>
#include <aikido/perception/YamlFileLoader.hpp>

namespace aikido{
namespace perception{


YamlFileLoader::YamlFileLoader(dart::common::ResourceRetrieverPtr& resourceRetriever,
                               dart::common::Uri configDataURI)
{
    dart::common::ResourceRetrieverPtr _resourceRetrieverPtr = resourceRetriever;

    //Read JSON file into string
    if(!_resourceRetrieverPtr)
        _resourceRetrieverPtr = std::make_shared<dart::common::LocalResourceRetriever>();

    const dart::common::ResourcePtr resource = _resourceRetrieverPtr->retrieve(configDataURI);
    if(!resource)
    {
        dtwarn<<"[ConfigDataLoader] Failed opening URI "<<configDataURI.toString()<<"\n";
        throw std::runtime_error("Failed opening URI.");
    }

    //Put file in string
    const size_t size = resource->getSize();
    std::string content;
    content.resize(size);
    if(resource->read(&content.front(), size, 1) != 1)
    {
        dtwarn << "[ConfigDataLoader] Failed reading from URI '"
            << configDataURI.toString() << "'.\n";
        throw std::runtime_error("Failed reading from URI.");
    }

    //Load from string
    mTagData = YAML::Load(content);

}

//Get name of object and pose for a given tag ID
bool YamlFileLoader::getTagNameOffset(const std::string _tagName, std::string& body_name, Eigen::Isometry3d& body_offset)
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


} //namespace perception

} //namespace aikido