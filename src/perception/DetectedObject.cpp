#include "aikido/perception/DetectedObject.hpp"

#include <yaml-cpp/exceptions.h>

namespace aikido {
namespace perception {

//==============================================================================
DetectedObject::DetectedObject(
    const std::string& objectName,
    const int& objectId,
    const std::string& detectionFrameID,
    const std::string& yamlStr)
  : mObjectName(objectName)
  , mUid(objectName + "_" + std::to_string(objectId))
  , mDetectionFrameID(std::move(detectionFrameID))
{
  // Load YAML nodes from string
  try
  {
    mYamlNode = YAML::Load(yamlStr);
    mAssetKey = mYamlNode["db_key"].as<std::string>();
  }
  catch (const YAML::Exception& e)
  {
    std::stringstream ss;
    ss << "[DetectedObject::DetectedObject] YAML String Exception: " << e.what()
       << std::endl;
    throw std::invalid_argument(ss.str());
  }
}

//==============================================================================
std::string DetectedObject::getUid() const
{
  return mUid;
}

//==============================================================================
std::string DetectedObject::getAssetKey() const
{
  return mAssetKey;
}

//==============================================================================
std::string DetectedObject::getDetectionFrameID() const
{
  return mDetectionFrameID;
}

//==============================================================================
std::string DetectedObject::getName() const
{
  return mObjectName;
}

//==============================================================================
YAML::Node DetectedObject::getYamlNode()
{
  return mYamlNode;
}

//==============================================================================
dart::dynamics::MetaSkeletonPtr DetectedObject::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

//==============================================================================
void DetectedObject::setMetaSkeleton(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton)
{
  mMetaSkeleton = metaSkeleton;
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
