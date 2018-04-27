#ifndef AIKIDO_IO_UTIL_HPP_
#define AIKIDO_IO_UTIL_HPP_

#include <aikido/io/CatkinResourceRetriever.hpp>
#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

using dart::dynamics::SkeletonPtr;

namespace aikido {
namespace io {

const SkeletonPtr makeBodyFromURDF(
    const std::shared_ptr<aikido::io::CatkinResourceRetriever>
        resourceRetriever,
    const std::string& uri,
    const Eigen::Isometry3d& transform)
{
  dart::utils::DartLoader urdfLoader;
  const SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri, resourceRetriever);

  if (!skeleton)
    throw std::runtime_error("unable to load '" + uri + "'");

  dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0))
      ->setTransform(transform);
  return skeleton;
}

} // namespace io
} // namespace aikido

#endif // AIKIDO_IO_UTIL_HPP_
