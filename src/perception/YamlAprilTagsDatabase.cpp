#include <aikido/perception/YamlAprilTagsDatabase.hpp>

#include <dart/common/Console.hpp>
#include <dart/common/LocalResourceRetriever.hpp>
#include <yaml-cpp/exceptions.h>

namespace aikido {
namespace perception {

//==============================================================================
YamlAprilTagsDatabase::YamlAprilTagsDatabase(
    const dart::common::ResourceRetrieverPtr& resourceRetriever,
    const dart::common::Uri& configDataURI)
{
  // Read JSON file into string
  if (!resourceRetriever)
  {
    throw std::invalid_argument("ResourceRetrieverPtr given is null!");
  }

  const dart::common::ResourcePtr resource
      = resourceRetriever->retrieve(configDataURI);
  if (!resource)
  {
    throw std::runtime_error(
        std::string("Failed opening URI - ") + configDataURI.toString());
  }

  // Put file in string
  const std::size_t size = resource->getSize();
  std::string content;
  content.resize(size);
  if (resource->read(&content.front(), size, 1) != 1)
  {
    throw std::runtime_error(
        std::string("Failed reading  URI - ") + configDataURI.toString());
  }

  // Load from string
  mTagData = YAML::Load(content);
}

//==============================================================================
bool YamlAprilTagsDatabase::getTagNameOffset(
    const std::string& tagName,
    std::string& bodyName,
    dart::common::Uri& bodyResource,
    Eigen::Isometry3d& bodyOffset)
{
  // Get name of object and pose for a given tag ID
  YAML::Node name_offset = mTagData[tagName];
  if (name_offset)
  {
    // Convert resource field
    try
    {
      bodyResource.fromString(name_offset["resource"].as<std::string>());
    }
    catch (const YAML::ParserException& ex)
    {
      throw std::runtime_error("Error in converting [resource] field");
    }

    // Convert name field
    try
    {
      bodyName = name_offset["name"].as<std::string>();
    }
    catch (const YAML::ParserException& ex)
    {
      throw std::runtime_error("Error in converting [name] field");
    }

    // Convert offset field
    try
    {
      bodyOffset = name_offset["offset"].as<Eigen::Isometry3d>();
    }
    catch (const YAML::ParserException& ex)
    {
      throw std::runtime_error("Error in converting [offset] field");
    }
    return true;
  }
  else
  {
    return false;
  }
}

} // namespace perception
} // namespace aikido
