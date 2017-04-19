#ifndef AIKIDO_UTIL_YAML_HPP_
#define AIKIDO_UTIL_YAML_HPP_

#include <yaml-cpp/yaml.h>

#include "aikido/util/detail/yaml_eigen_extension.hpp"
// The above header allows extended value types for yaml-cpp.
//
// Supported types:
// - Eigen::Matrix<...>
// - Eigen::Isometry<...>
// - std::unordered_map
//

#endif // AIKIDO_UTIL_YAML_HPP_
