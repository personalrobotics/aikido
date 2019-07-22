#include "aikido/perception/AssetDatabase.hpp"

#include <dart/common/Console.hpp>
#include <dart/common/LocalResourceRetriever.hpp>
#include <yaml-cpp/exceptions.h>

namespace aikido {
namespace perception {

//==============================================================================
AssetDatabase::AssetDatabase(
    const dart::common::ResourceRetrieverPtr& resourceRetriever,
    const dart::common::Uri& configDataURI)
{
  // Read the JSON file into string
  if (!resourceRetriever)
  {
    throw std::invalid_argument("ResourceRetriever is null");
  }

  std::string content = resourceRetriever->readAll(configDataURI);
  if (content.empty())
  {
    throw std::runtime_error(
        std::string("Failed load URI - ") + configDataURI.toString());
  }

  // Load from string
  try
  {
    mAssetData = YAML::Load(content);
  }
  catch (YAML::Exception& e)
  {
    dtwarn << "[AssetDatabase::AssetDatabase] JSON File Exception: " << e.what()
           << std::endl
           << "Loading empty asset database." << std::endl;
    mAssetData = YAML::Load(""); // Create Null Node
  }
}

//==============================================================================
void AssetDatabase::getAssetByKey(
    const std::string& assetKey,
    dart::common::Uri& assetResource,
    Eigen::Isometry3d& assetOffset) const
{
  // Get name of asset and pose for a given tag ID
  YAML::Node assetNode = mAssetData[assetKey];
  if (!assetNode)
  {
    throw std::runtime_error(
        "[AssetDatabase] Error: invalid asset key: " + assetKey);
  }

  // Convert resource field
  try
  {
    assetResource.fromString(assetNode["resource"].as<std::string>());
  }
  catch (const YAML::ParserException& ex)
  {
    throw std::runtime_error(
        "[AssetDatabase] Error in converting [resource] field");
  }

  // Convert offset field
  try
  {
    assetOffset = assetNode["offset"].as<Eigen::Isometry3d>();
  }
  catch (const YAML::ParserException& ex)
  {
    throw std::runtime_error(
        "[AssetDatabase] Error in converting [offset] field");
  }
}

} // namespace perception
} // namespace aikido
