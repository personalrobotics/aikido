#ifndef AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_
#define AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_

#include <string>
#include "aikido/io/yaml.hpp"

namespace aikido {
namespace perception {

class DetectedObject
{
public:
  DetectedObject(
      const std::string& objDBKey,
      const std::string& detectionFrameID,
      const std::string& yamlStr);

  virtual ~DetectedObject() = default;

  std::string getObjUID();

  std::string getObjDBKey();

  std::string getDetectionFrameID();

  YAML::Node getYamlNode();

  template <typename T>
  T getInfoByKey(const std::string& key);

private:
  std::string mObjUID;

  std::string mObjDBKey;

  std::string mDetectionFrameID;

  YAML::Node mYamlNode;
};

} // namespace perception
} // namespace aikido

#endif // AIKIDO_PERCEPTION_DETECTEDOBJECT_HPP_
