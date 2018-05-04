#include "aikido/io/util.hpp"

namespace aikido {
namespace io {

//==============================================================================
dart::dynamics::SkeletonPtr loadSkeletonFromURDF(
    const dart::common::ResourceRetrieverPtr resourceRetriever,
    const std::string& uri,
    const Eigen::Isometry3d& transform)
{
  dart::utils::DartLoader urdfLoader;
  const dart::dynamics::SkeletonPtr skeleton
      = urdfLoader.parseSkeleton(uri, resourceRetriever);

  if (!skeleton)
    throw std::runtime_error("Unable to load '" + uri + "'");

  if (skeleton->getNumJoints() <= 0)
  {
    throw std::runtime_error("Skeleton is empty.");
  }

  auto freeJoint
      = dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getRootJoint(0));

  if (freeJoint == nullptr)
  {
    throw std::runtime_error(
        "Unable to cast Skeleton's root joint to FreeJoint.");
  }
  freeJoint->setTransform(transform);

  return skeleton;
}

} // namespace io
} // namespace aikido
