#ifndef AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_
#define AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_

#include <string>
#include <dart/dart.hpp>
#include "aikido/io/yaml.hpp"

namespace aikido {
namespace perception {

/// DetectedObject delegates a detected object from a third-party
/// perception algorithm.

/// A perception algorithm should send information for an object via ROS
/// visualization_msgs/Marker message like the following:
/// Marker.ns + "_" + Marker.id -> uid (identity in planner::World)
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

  /// Default constructor.
  DetectedObject() = default;

  /// Construct a \c DetectedObject
  /// \param[in] uid Unique ID for object in Aikido world. Same UID -> Same
  /// Object
  /// \param[in] detectionFrameID Frame ID from ROS Marker
  /// \param[in] yamlStr String of additional parameters for object. Can
  /// override objAssetDBKey by specifying "db_key".
  DetectedObject(
      const std::string& objectName,
      const int& objectId,
      const std::string& detectionFrameID,
      const std::string& yamlStr);

  virtual ~DetectedObject() = default;

  /// Get the unique  id of the object
  std::string getUid() const;

  /// Get the object key for \c AssetDatabase
  std::string getAssetKey() const;

  /// Get the detection frame id that refers the origin of this object's pose
  std::string getDetectionFrameID() const;

  /// Get the name of this object.
  std::string getName() const;

  /// Get the map of keys to additional informations
  YAML::Node getYamlNode();

  /// Get the metaSkeleton associated with this object.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const;

  /// Set Metaskeleton.
  void setMetaSkeleton(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton);

  /// Get a specific value from the information map by a key and the typename
  /// of the field
  /// \param[in] key The key (string) of a field in the information map
  /// Sequence types (e.g. [1, 2]) can be read into standard containers (e.g.
  /// std::vector<double>)
  /// Map types are not supported with this function. Please get the manually
  /// with getYamlNode().
  template <typename T>
  T getInfoByKey(const std::string& key);

private:

  /// The name of the object.
  std::string mObjectName;

  /// The unique id of the object
  std::string mUid;

  /// The object key for \c AssetDatabase
  std::string mAssetKey;

  /// The detection frame id that refers the origin of this object's pose
  std::string mDetectionFrameID;

  /// The information map with additional information of this object
  YAML::Node mYamlNode;

  /// MetaSkeleton associated with this object.
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_
