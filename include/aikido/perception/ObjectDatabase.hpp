#ifndef AIKIDO_PERCEPTION_OBJECT_DATABASE_HPP_
#define AIKIDO_PERCEPTION_OBJECT_DATABASE_HPP_

#include <stdexcept>
#include <dart/common/LocalResourceRetriever.hpp>
#include <dart/dart.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/yaml.hpp>

namespace aikido {
namespace perception {

/// Instantiation of ObjectDatabase that reads of JSON file containing the
/// information that maps object keys to the object names and resources.

/// The JSON file should have a map with object keys.
/// Each such key points to a nested map, where the keys are
/// 'resource' and 'name'.
/// The values for each of the nested keys for a particular
/// object key are to be returned to the calling method
/// via the callback.
///
/// Here is an example entry in a JSON file:
/// \code
/// "obj_key": {
///    "resource": "package://pr_ordata/data/objects/obj_filename.urdf",
///    "name": "obj_name"
///  }
/// \endcode

class ObjectDatabase
{
public:
  /// Construct a \c ObjectDatabase that uses \c ResourceRetriever to
  /// load configuration data from a JSON file at URI \c configDataURI.
  /// \param[in] resourceRetriever The pointer to obtain the configuration file
  /// \param[in] configDataURI The URI for the configuration information file
  ObjectDatabase(
      const dart::common::ResourceRetrieverPtr& resourceRetriever,
      dart::common::Uri configDataURI);

  virtual ~ObjectDatabase() = default;

  bool getObjectByKey(
      const std::string& _obj_key,
      std::string& obj_name,
      dart::common::Uri& obj_resource);

private:
  /// The map of object keys to object names and resources for models
  YAML::Node mObjData;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_OBJECT_DATABASE_HPP_
