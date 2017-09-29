#ifndef AIKIDO_PERCEPTION_YAML_APRILTAGS_DATABASE_HPP_
#define AIKIDO_PERCEPTION_YAML_APRILTAGS_DATABASE_HPP_

#include <stdexcept>
#include <Eigen/Geometry>
#include <dart/common/LocalResourceRetriever.hpp>
#include <dart/dart.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/yaml.hpp>
#include "AprilTagsDatabase.hpp"

namespace aikido {
namespace perception {

/// Instantiation of AprilTagsDatabase that reads a YAML file containing the
/// information that maps tag IDs to the object name, resource and relative
/// transform.
///
/// The YAML file should have a map with keys being the names of the expected
/// tags. Each such key points to a nested map, where the keys are 'resource',
/// 'name' and 'offset'. The values for each of the nested keys for a particular
/// tag ID are to be returned to the calling method via the callback.
///
/// Here is an example entry in a JSON (subset of YAML) file:
/// \code
/// "tag124": {
///    "resource": "package://pr_ordata/data/objects/plastic_glass.urdf",
///    "name": "plastic_glass",
///    "offset": [
///      [
///        -0.0068393994632501,
///        -0.09054787493933,
///        -0.99586861832219,
///        0.14460904118031
///        ]
///      ,
///      [
///        -0.9999766064057,
///        0.00052349074982687,
///        0.0068200145734176,
///        -0.00097599685803373
///        ]
///      ,
///      [
///        -9.6209816943679e-5,
///        0.99589196617977,
///        -0.090549337061432,
///        -0.033644917605727
///      ]
///      ,
///      [
///        0,
///        0,
///        0,
///        1
///      ]
///    ]
///  }
/// \endcode
class YamlAprilTagsDatabase : public AprilTagsDatabase
{
public:
  /// Construct a \c YamlAprilTagsDatabase that uses \c ResourceRetriever to
  /// load configuration data from a YAML file at URI \c configDataURI.
  /// \param[in] resourceRetriever The pointer to obtain the configuration file
  /// \param[in] configDataURI The URI for the configuration information file
  YamlAprilTagsDatabase(
      const dart::common::ResourceRetrieverPtr& resourceRetriever,
      dart::common::Uri configDataURI);

  virtual ~YamlAprilTagsDatabase() = default;

  // Documentation inherited
  bool getTagNameOffset(
      const std::string& _tagName,
      std::string& body_name,
      dart::common::Uri& body_resource,
      Eigen::Isometry3d& body_offset) override;

private:
  /// The map of tag IDs to object name, resource for model and offset
  YAML::Node mTagData;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_YAML_APRILTAGS_DATABASE_HPP_
