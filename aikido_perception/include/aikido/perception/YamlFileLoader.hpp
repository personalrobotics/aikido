/** 
 * @file YamlFileLoader.hpp
 * @author Shushman Choudhury
 * @date Apr 20, 2016
 * @brief The header for the instance of the configuration data loader 
 * for marker-based perception which reads a YAML file.
 */

#ifndef AIKIDO_PERCEPTION_YAML_FILE_LOADER_H
#define AIKIDO_PERCEPTION_YAML_FILE_LOADER_H

#include "yaml-cpp/yaml.h"
#include <aikido/perception/eigen_yaml.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <dart/common/LocalResourceRetriever.h>
#include <dart/dart.h>
#include <Eigen/Geometry>
#include "ConfigDataLoader.hpp"
#include <stdexcept>

namespace aikido{
namespace perception{

/// The YAML File configuration data loader
class YamlFileLoader : public virtual ConfigDataLoader
{
public:
    YamlFileLoader(const dart::common::ResourceRetrieverPtr& resourceRetriever,
                   dart::common::Uri configDataURI);

    virtual ~YamlFileLoader() = default;

    ///Lookup tag name and return name of body, resource to URDF file and offset of body
    bool getTagNameOffset(const std::string& _tagName, std::string& body_name, dart::common::Uri& body_resource, Eigen::Isometry3d& body_offset) override;

private:

    ///The map of tag IDs to object name, resource for model and offset
    YAML::Node mTagData;
};


} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_YAML_FILE_LOADER_H