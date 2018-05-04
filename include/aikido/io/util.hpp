#ifndef AIKIDO_IO_UTIL_HPP_
#define AIKIDO_IO_UTIL_HPP_

#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include "aikido/io/CatkinResourceRetriever.hpp"

namespace aikido {
namespace io {

/// Loads a Skeleton from an URDF file and returns it
/// \param[in] resourceRetriever Passed to an URDFLoader object to get the resource
/// \param [in] uri The URI where the URDF can be found
/// \param [in] transform The initial transform of the skeleton
/// \return A pointer to the created Skeleton
dart::dynamics::SkeletonPtr loadSkeletonFromURDF(
    const dart::common::ResourceRetrieverPtr resourceRetriever,
    const std::string& uri,
    const Eigen::Isometry3d& transform = Eigen::Isometry3d::Identity());

} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_UTIL_HPP_
