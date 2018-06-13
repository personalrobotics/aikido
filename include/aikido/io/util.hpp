#ifndef AIKIDO_IO_UTIL_HPP_
#define AIKIDO_IO_UTIL_HPP_

#include <dart/dart.hpp>

namespace aikido {
namespace io {

/// Load a DART Skeleton from a URDF and set its pose.
///
/// \param[in] retriever DART retriever to resolve the URI
/// \param[in] uri URI to the object URDF
/// \param[in] transform Initial transform for the Skeleton
/// \return the created Skeleton
dart::dynamics::SkeletonPtr loadSkeletonFromURDF(
    const dart::common::ResourceRetrieverPtr& retriever,
    const dart::common::Uri& uri,
    const Eigen::Isometry3d& transform = Eigen::Isometry3d::Identity());

} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_UTIL_HPP_
