#include "aikido/perception/DetectedObject.hpp"

#include <yaml-cpp/exceptions.h>

namespace aikido {
namespace perception {

//==============================================================================
DetectedObject::DetectedObject(
    const std::string& dartUid,
    const std::string& assetKey,
    const std::string& detectionFrameID,
    const std::string& yamlStr)
  : mDartUid(std::move(dartUid))
  , mAssetKey(std::move(assetKey))
  , mDetectionFrameID(std::move(detectionFrameID))
{
  // Load YAML nodes from string
  try
  {
    mYamlNode = YAML::Load(yamlStr);
    mAssetKey = mYamlNode["db_key"].as<std::string>(mAssetKey);
  }
  catch (const YAML::Exception& e)
  {
    dtwarn << "[DetectedObject::DetectedObject] YAML String Exception: "
           << e.what() << std::endl;
    mYamlNode = YAML::Load(""); // Create Null Node
  }
}

//==============================================================================
std::string DetectedObject::getDartUid()
{
  return mDartUid;
}

//==============================================================================
std::string DetectedObject::getAssetKey()
{
  return mAssetKey;
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
