#include <dart/common/Console.hpp>
#include <dart/common/LocalResourceRetriever.hpp>
#include <yaml-cpp/exceptions.h>
#include <aikido/perception/YamlAprilTagsDatabase.hpp>

using namespace std;

namespace aikido {
namespace perception {

//==============================================================================
YamlAprilTagsDatabase::YamlAprilTagsDatabase(
    const dart::common::ResourceRetrieverPtr& resourceRetriever,
    dart::common::Uri configDataURI)
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
  const size_t size = resource->getSize();
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
    const std::string& _tagName,
    std::string& body_name,
    dart::common::Uri& body_resource,
    Eigen::Isometry3d& body_offset)
{
  // Get name of object and pose for a given tag ID
  YAML::Node name_offset = mTagData[_tagName];
  if (name_offset)
  {

    // Convert resource field
    try
    {
      body_resource.fromString(name_offset["resource"].as<std::string>());
    }
    catch (const YAML::ParserException& ex)
    {
      throw std::runtime_error("Error in converting [resource] field");
    }

    // Convert name field
    try
    {
      body_name = name_offset["name"].as<std::string>();
    }
    catch (const YAML::ParserException& ex)
    {
      throw std::runtime_error("Error in converting [name] field");
    }

    // Convert offset field
    try
    {
      body_offset = name_offset["offset"].as<Eigen::Isometry3d>();
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
