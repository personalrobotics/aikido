#ifndef AIKIDO_PERCEPTION_ASSET_DATABASE_HPP_
#define AIKIDO_PERCEPTION_ASSET_DATABASE_HPP_

#include <stdexcept>
#include <dart/common/LocalResourceRetriever.hpp>
#include <dart/dart.hpp>
#include "aikido/io/CatkinResourceRetriever.hpp"
#include "aikido/io/yaml.hpp"

namespace aikido {
namespace perception {

/// Instantiation of AssetDatabase that reads of JSON file containing the
/// information that maps object keys to visual assets and resources.

/// The JSON file should have a map with object keys.
/// Each such key points to a nested map, where the keys are
/// 'resource' and 'name'.
/// The values for each of the nested keys for a particular
/// object key are to be returned to the calling method
/// via the callback.
///
/// Here is an example entry in a JSON file:
/// \code
/// "asset_key": {
///    "resource": "package://pr_assets/data/objects/obj_filename.urdf",
///    "name": "asset_name",
///    "offset": [
///      [1.0, 0.0, 0.0, 0.0],
///      [0.0, 1.0, 0.0, 0.0],
///      [0.0, 0.0, 1.0, 0.0],
///      [0.0, 0.0, 0.0, 1.0]
///    ]
///  }
/// \endcode

class AssetDatabase
{
public:
  /// Construct a \c AssetDatabase that uses \c ResourceRetriever to
  /// load configuration data from a JSON file at URI \c configDataURI.
  /// \param[in] resourceRetriever The pointer to obtain the configuration file
  /// \param[in] configDataURI The URI for the configuration information file
  AssetDatabase(
      const dart::common::ResourceRetrieverPtr& resourceRetriever,
      const dart::common::Uri& configDataURI);

  virtual ~AssetDatabase() = default;

  /// Get the object name, resource, and offset from database by objectKey
  /// \param[in]  assetKey The key (string) of an object in AssetDatabase
  /// \param[out] assetName The retrieved object name from AssetDatabase
  /// \param[out] assetResource The retrieved uri of the object
  /// \param[out] assetOffset The retrieved offset matrix of the object
  ///     i.e. the offset between a tag and the actual origin of an object
  void getAssetByKey(
      const std::string& assetKey,
      std::string& assetName,
      dart::common::Uri& assetResource,
      Eigen::Isometry3d& assetOffset) const;

private:
  /// The map of asset keys to object names and resources for models
  YAML::Node mAssetData;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_OBJECT_DATABASE_HPP_
