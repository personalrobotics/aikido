#include <aikido/perception/ObjectDatabase.hpp>

#include <dart/common/Console.hpp>
#include <dart/common/LocalResourceRetriever.hpp>
#include <yaml-cpp/exceptions.h>

namespace aikido {
namespace perception {

//==============================================================================
ObjectDatabase::ObjectDatabase(
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
  mObjData = YAML::Load(content);
}

//==============================================================================
void ObjectDatabase::getObjectByKey(
    const std::string& objectKey,
    std::string& objectName,
    dart::common::Uri& objectResource,
    Eigen::Isometry3d& objectOffset) const
{
  // Get name of object and pose for a given tag ID
  YAML::Node objectNode = mObjData[objectKey];
  if (!objectNode)
  {
    throw std::runtime_error("[ObjectDatabase] Error: invalid object key");
  }

  // Convert resource field
  try
  {
    objectResource.fromString(objectNode["resource"].as<std::string>());
  }
  catch (const YAML::ParserException& ex)
  {
    throw std::runtime_error(
        "[ObjectDatabase] Error in converting [resource] field");
  }

  // Convert name field
  try
  {
    objectName = objectNode["name"].as<std::string>();
  }
  catch (const YAML::ParserException& ex)
  {
    throw std::runtime_error(
        "[ObjectDatabase] Error in converting [name] field");
  }

  // Convert offset field
  try
  {
    objectOffset = objectNode["offset"].as<Eigen::Isometry3d>();
  }
  catch (const YAML::ParserException& ex)
  {
    throw std::runtime_error(
        "[ObjectDatabase] Error in converting [offset] field");
  }
}

} // namespace perception
} // namespace aikido
