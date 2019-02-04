#ifndef AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_
#define AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_

#include <string>
#include "aikido/io/yaml.hpp"

namespace aikido {
namespace perception {

/// DetectedObject delegates a detected object from a third party
/// perception algorithm.

/// A perception algorithm should send information for an object via ROS
/// visualization_msgs/Marker message like following:
/// Marker.header.frame_id -> detectionFrameID
/// Marker.ns -> objDBKey
/// Marker.text -> yamlStr

/// yamlStr contains the unique id of the object and any types of additional
/// informations of the object in JSON format.
/// Here is an example:
/// \code
/// {
///   "uid": "cantaloupe_001",
///   "score": 0.9,
///   "success_rates": [0.4, 0.9, 0.1, 0.2],
///   "strategies": ["vertical_0", "vertical_90", "tilted_0", "tilted_90"],
/// }
/// \endcode

class DetectedObject
{
public:
  /// Construct a \c DetectedObject
  /// \param[in] objDBKey
  /// \param[in] detectionFrameID
  /// \param[in] yamlStr
  DetectedObject(
      const std::string& objDBKey,
      const std::string& detectionFrameID,
      const std::string& yamlStr);

  virtual ~DetectedObject() = default;

  /// Get the unique id of the object
  std::string getObjUID();

  /// Get the object key for \c ObjectDatabase
  std::string getObjDBKey();

  /// Get the detection frame id that refers the origin of this object's pose
  std::string getDetectionFrameID();

  /// Get the map of keys to additional informations
  YAML::Node getYamlNode();

  /// Get a specific value from the information map by a key and the typename
  /// of the field
  /// \param[in] key The key (string) of a field in the information map
  template <typename T>
  T getInfoByKey(const std::string& key);

private:
  /// The unique id of the object
  std::string mObjUID;

  /// The object key for \c ObjectDatabase
  std::string mObjDBKey;

  /// The detection frame id that refers the origin of this object's pose
  std::string mDetectionFrameID;

  /// The information map with additional informations of this object
  YAML::Node mYamlNode;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_
