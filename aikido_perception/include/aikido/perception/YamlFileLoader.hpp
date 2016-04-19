#ifndef AIKIDO_PERCEPTION_YAML_FILE_LOADER_H
#define AIKIDO_PERCEPTION_YAML_FILE_LOADER_H

#include "yaml-cpp/yaml.h"
#include <aikido/perception/yaml_conversion.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <dart/common/LocalResourceRetriever.h>
#include <dart/dart.h>
#include <Eigen/Geometry>
#include "ConfigDataLoader.hpp"
#include <stdexcept>

namespace aikido{
namespace perception{

class YamlFileLoader : public virtual ConfigDataLoader
{
public:
    YamlFileLoader(dart::common::ResourceRetrieverPtr& resourceRetriever,
                   dart::common::Uri configDataURI);

    virtual ~YamlFileLoader() = default;

    bool getTagNameOffset(const std::string _tagName, std::string& body_name, Eigen::Isometry3d& body_offset);

private:

    //Member variables
    YAML::Node mTagData;
};


} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_YAML_FILE_LOADER_H