#include "aikido/io/yaml.hpp"

namespace aikido {
namespace io {

YAML::Node loadYAML(
    const dart::common::Uri& yamlUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  // These can be replaced with retriever::readAll(uri) with Dart 6.2.
  // std::string content = retriever->readAll(yamlUri);
  const auto resource = retriever->retrieve(yamlUri);
  const auto resourceSize = resource->getSize();
  std::string content;
  content.resize(resourceSize);
  if (resource->read(&content.front(), resourceSize, 1) != 1)
  {
    std::stringstream message;
    message << "Unable to load from '" << yamlUri.getPath() << "'.";
    throw std::runtime_error(message.str());
  }

  return YAML::Load(content);
}

} // namespace io
} // namespace aikido
