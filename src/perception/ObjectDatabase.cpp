#include <aikido/perception/ObjectDatabase.hpp>

#include <dart/common/Console.hpp>
#include <dart/common/LocalResourceRetriever.hpp>
#include <yaml-cpp/exceptions.h>

namespace aikido {
namespace perception {

//==============================================================================
ObjectDatabase::ObjectDatabase(
    const dart::common::ResourceRetrieverPtr& resourceRetriever,
    const dart::common::Uri configDataURI)
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
    const std::string& _obj_key,
    std::string& obj_name,
    dart::common::Uri& obj_resource) const
{
  // Get name of object and pose for a given tag ID
  YAML::Node obj_node = mObjData[_obj_key];
  if (!obj_node)
  {
    throw std::runtime_error("[ObjectDatabase] Error: invalid object key");
  }

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
}

} // namespace perception
} // namespace aikido
