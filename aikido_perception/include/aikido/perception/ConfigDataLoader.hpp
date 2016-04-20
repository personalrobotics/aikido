#ifndef AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H
#define AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H

#include <Eigen/Dense>
#include <dart/dart.h>

namespace aikido{
namespace perception{

class ConfigDataLoader
{
public:
    virtual ~ConfigDataLoader() = default;

    //Get name of object and pose for a given tag ID
    virtual bool getTagNameOffset(const std::string& _tagName, std::string& body_name, dart::common::Uri& body_resource, Eigen::Isometry3d& body_offset) = 0;

};


} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H