#ifndef AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_
#define AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_

#include <string>
#include "aikido/io/yaml.hpp"

namespace aikido {
namespace perception {

/// DetectedObject delegates a detected object from a third-party
/// perception algorithm.

/// A perception algorithm should send information for an object via ROS
/// visualization_msgs/Marker message like the following:
/// Marker.ns + "_" + Marker.id -> dartUid (identity in DART world)
/// Marker.ns -> assetKey (determines visual asset, \see AssetDatabase)
/// Marker.header.frame_id -> detectionFrameID
/// Marker.text -> yamlStr

/// yamlStr contains any additional information in YAML format.
/// It also can overwrite the Asset Database Key
/// using "db_key". (\see AssetDatabase for details)
/// Here is an example:
/// \code
/// {
///   "db_key": "food_item",
///   "score": 0.9,
///   "success_rates": [0.4, 0.9, 0.1, 0.2],
///   "strategies": ["vertical_0", "vertical_90", "tilted_0", "tilted_90"],
/// }
/// \endcode

class DetectedObject
{
public:
  /// Construct a \c DetectedObject
  /// \param[in] dartUid Unique ID for object in DART world. Same UID -> Same Object
  /// \param[in] assetKey Key for AssetDatabase passed into constructor of PoseEstimatorModule. Defines visuals / assets.
  /// \param[in] detectionFrameID Frame ID from ROS Marker
  /// \param[in] yamlStr String of additional parameters for object. Can override objAssetDBKey by specifying "db_key".
  DetectedObject(
      const std::string& dartUid,
      const std::string& assetKey,
      const std::string& detectionFrameID,
      const std::string& yamlStr);

  virtual ~DetectedObject() = default;

  /// Get the unique DART id of the object
  std::string getDartUid();

  /// Get the object key for \c AssetDatabase
  std::string getAssetKey();

  /// Get the detection frame id that refers the origin of this object's pose
  std::string getDetectionFrameID();

  /// Get the map of keys to additional informations
  YAML::Node getYamlNode();

  /// Get a specific value from the information map by a key and the typename
  /// of the field
  /// \param[in] key The key (string) of a field in the information map
  /// Sequence types (e.g. [1, 2]) can be read into standard containers (e.g. std::vector<double>)
  /// Map types are not supported with this function. Please get the manually with getYamlNode().
  template <typename T>
  T getInfoByKey(const std::string& key);

private:
  /// The unique id of the object
  std::string mDartUid;

  /// The object key for \c AssetDatabase
  std::string mAssetKey;

  /// The detection frame id that refers the origin of this object's pose
  std::string mDetectionFrameID;

  /// The information map with additional information of this object
  YAML::Node mYamlNode;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_
