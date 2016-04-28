#ifndef AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H
#define AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H

#include <Eigen/Dense>
#include <dart/dart.h>

namespace aikido{
namespace perception{

/// The interface for the configuration data loader.
class ConfigDataLoader
{
public:
    /// The virtual destructor
    virtual ~ConfigDataLoader() = default;

    /// A pure virtual method that is the callback used by the perception module
    /// \param[in] _tagName the ID of the tag to look up
    /// \param[out] body_name the name of the object associated with the tag
    /// \param[out] body_resource the resource that points to the URDF file of the object
    /// \param[out] body_offset the relative transform between the tag and the object it is attached to
    /// \return the name, file resource and relative transform of the body with respect to the tag
    virtual bool getTagNameOffset(const std::string& _tagName, std::string& body_name, dart::common::Uri& body_resource, Eigen::Isometry3d& body_offset) = 0;

};


} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H