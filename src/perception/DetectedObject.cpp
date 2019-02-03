#include "aikido/perception/DetectedObject.hpp"

#include <yaml-cpp/exceptions.h>

namespace aikido {
namespace perception {

//==============================================================================
DetectedObject::DetectedObject(
    const std::string& objDBKey,
    const std::string& detectionFrameID,
    const std::string& yamlStr)
  : mObjDBKey(std::move(objDBKey))
  , mDetectionFrameID(std::move(detectionFrameID))
{
  // Load YAML nodes from string
  mYamlNode = YAML::Load(yamlStr);

  mObjUID = mYamlNode["id"].as<std::string>();
}

//==============================================================================
std::string DetectedObject::getObjUID()
{
  return mObjUID;
}

//==============================================================================
std::string DetectedObject::getObjDBKey()
{
  return mObjDBKey;
}

//==============================================================================
std::string DetectedObject::getDetectionFrameID()
{
  return mDetectionFrameID;
}

//==============================================================================
YAML::Node DetectedObject::getYamlNode()
{
  return mYamlNode;
}

//==============================================================================
template <typename T>
T DetectedObject::getInfoByKey(const std::string& key)
{
  T value;
  try
  {
    value = mYamlNode[key].as<T>();
  }
  catch (const YAML::ParserException& ex)
  {
    throw std::runtime_error(
        "[DetectedObject] Error in converting [" + key + "] field");
  }
  return value;
}

} // namespace perception
} // namespace aikido
