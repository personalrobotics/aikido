#ifndef AIKIDO_IO_UTIL_HPP_
#define AIKIDO_IO_UTIL_HPP_

#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include "aikido/io/CatkinResourceRetriever.hpp"

namespace aikido {
namespace io {

dart::dynamics::SkeletonPtr loadSkeletonFromURDF(
    const dart::common::ResourceRetrieverPtr resourceRetriever,
    const std::string& uri,
    const Eigen::Isometry3d& transform);

} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_UTIL_HPP_
