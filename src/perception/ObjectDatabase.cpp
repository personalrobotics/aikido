#include <aikido/perception/ObjectDatabase.hpp>

#include <dart/common/Console.hpp>
#include <dart/common/LocalResourceRetriever.hpp>
#include <yaml-cpp/exceptions.h>

namespace aikido {
namespace perception {

//==============================================================================
ObjectDatabase::ObjectDatabase(
    const dart::common::ResourceRetrieverPtr& resourceRetriever,
    dart::common::Uri configDataURI)
{
  // Read the JSON file into string
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
  mObjData = YAML::Load(content);
}

//==============================================================================
bool ObjectDatabase::getObjectByKey(
    const std::string& _obj_key,
    std::string& obj_name,
    dart::common::Uri& obj_resource)
{
  // Get name of object and pose for a given tag ID
  YAML::Node obj_node = mObjData[_obj_key];
  if (obj_node)
  {
    // Convert resource field
    try
    {
      obj_resource.fromString(obj_node["resource"].as<std::string>());
    }
    catch (const YAML::ParserException& ex)
    {
      throw std::runtime_error(
          "[ObjectDatabase] Error in converting [resource] field");
    }

    // Convert name field
    try
    {
      obj_name = obj_node["name"].as<std::string>();
    }
    catch (const YAML::ParserException& ex)
    {
      throw std::runtime_error(
          "[ObjectDatabase] Error in converting [name] field");
    }

    return true;
  }
  else
  {
    throw std::runtime_error("[ObjectDatabase] Error: invalid object key");
    return false;
  }
}

} // namespace perception
} // namespace aikido
