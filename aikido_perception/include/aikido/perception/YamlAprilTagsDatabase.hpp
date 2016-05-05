#ifndef AIKIDO_PERCEPTION_YAML_APRILTAGS_DATABASE_H
#define AIKIDO_PERCEPTION_YAML_APRILTAGS_DATABASE_H

#include "yaml-cpp/yaml.h"
#include <aikido/perception/eigen_yaml.hpp>
#include <aikido/util/CatkinResourceRetriever.hpp>
#include <dart/common/LocalResourceRetriever.h>
#include <dart/dart.h>
#include <Eigen/Geometry>
#include "AprilTagsDatabase.hpp"
#include <stdexcept>

namespace aikido{
namespace perception{

/// Instantiation of AprilTagsDatabase that reads a YAML file containing the information
/// that maps tag IDs to the object name, resource and relative transform
/// The YAML file should have a map with keys being the names of the expected tags
/// Each such key points to a nested map, where the keys are 'resource', 'name' and 'offset'
/// The values for each of the nested keys for a particular Tag ID are to be returned to the
/// calling method via the callback.
/// 
class YamlAprilTagsDatabase : public AprilTagsDatabase
{
public:

    /// Construct a \c YamlAprilTagsDatabase that uses \c ResourceRetriever to
    /// load configuration data from a YAML file at URI \c configDataURI.
    /// \param[in] resourceRetriever the pointer to obtain the configuration file
    /// \param[in] configDataURI the URI for the configuration information file
    YamlAprilTagsDatabase(const dart::common::ResourceRetrieverPtr& resourceRetriever,
                   dart::common::Uri configDataURI);

    virtual ~YamlAprilTagsDatabase() = default;

    // Documentation inherited
    bool getTagNameOffset(const std::string& _tagName, std::string& body_name, dart::common::Uri& body_resource, Eigen::Isometry3d& body_offset) override;

private:

    ///The map of tag IDs to object name, resource for model and offset
    YAML::Node mTagData;
};


} //namespace perception

} //namespace aikido

#endif //AIKIDO_PERCEPTION_YAML_APRILTAGS_DATABASE_H