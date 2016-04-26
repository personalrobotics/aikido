/** 
 * @file ConfigDataLoader.hpp
 * @author Shushman Choudhury
 * @date Apr 24, 2016
 * @brief The interface for the configuration data loader.
 */

#ifndef AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H
#define AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H

#include <Eigen/Dense>
#include <dart/dart.h>

namespace aikido{
namespace perception{

class ConfigDataLoader
{
public:
    //! The virtual destructor
    virtual ~ConfigDataLoader() = default;

    //! A pure virtual method that is the callback used by the perception module
    /*!
        \param _tagName the ID of the tag to look up
        \param body_name the name of the object associated with the tag
        \param body_resource the resource that points to the URDF file of the object
        \param body_offset the relative transform between the tag and the object it is attached to
    */
    virtual bool getTagNameOffset(const std::string& _tagName, std::string& body_name, dart::common::Uri& body_resource, Eigen::Isometry3d& body_offset) = 0;

};


} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_CONFIG_DATA_LOADER_H