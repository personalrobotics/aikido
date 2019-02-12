#include "aikido/perception/DetectedObject.hpp"

#include <yaml-cpp/exceptions.h>

namespace aikido {
namespace perception {

//==============================================================================
DetectedObject::DetectedObject(
    const std::string& objUID,
    const std::string& detectionFrameID,
    const std::string& yamlStr)
  : mObjDBKey(std::move(objUID))
  , mDetectionFrameID(std::move(detectionFrameID))
{
  // Load YAML nodes from string
  try {
    mYamlNode = YAML::Load(yamlStr);
    mObjDBKey = mYamlNode["db_key"].as<std::string>(mObjDBKey);
  } catch(const YAML::Exception &e) {
    mObjDBKey = "";
    dtwarn << "[DetectedObject::DetectedObject] YAML String Exception: " << e.what() << std::endl;
  }
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
