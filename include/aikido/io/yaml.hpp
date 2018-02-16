#ifndef AIKIDO_IO_YAML_HPP_
#define AIKIDO_IO_YAML_HPP_

#include <yaml-cpp/yaml.h>
#include <dart/dart.hpp>

// This header allows extended value types for yaml-cpp.
//
// Supported types:
// - Eigen::Matrix<...>
// - Eigen::Isometry<...>
// - std::unordered_map
#include "aikido/io/detail/yaml_extension.hpp"

namespace aikido {
namespace io {

/// Retrieves and read all data from a YAML file.
/// \param[in] yamlUri URI for a YAML file
/// \param[in] retriever Resource retriever to retrieve yamlUri
/// \return root YAML::Node of the document
YAML::Node loadYAML(
    const dart::common::Uri& yamlUri,
    const dart::common::ResourceRetrieverPtr& retriever);

} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_YAML_HPP_
