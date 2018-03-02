#include "aikido/io/yaml.hpp"

namespace aikido {
namespace io {

YAML::Node loadYAML(
    const dart::common::Uri& yamlUri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
  std::string content = retriever->readAll(yamlUri);
  return YAML::Load(content);
}

} // namespace io
} // namespace aikido
