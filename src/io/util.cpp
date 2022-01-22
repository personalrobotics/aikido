#include "aikido/io/util.hpp"

#include <dart/utils/urdf/DartLoader.hpp>

#include "aikido/io/CatkinResourceRetriever.hpp"

namespace aikido {
namespace io {

//==============================================================================
dart::dynamics::SkeletonPtr loadSkeletonFromURDF(
    const dart::common::ResourceRetrieverPtr& retriever,
    const dart::common::Uri& uri,
    const Eigen::Isometry3d& transform)
{
#if DART_VERSION_AT_LEAST(6, 12, 0)
  dart::utils::DartLoader::Options options;
  options.mResourceRetreiver = retriever;
  dart::utils::DartLoader urdfLoader(options);
  const dart::dynamics::SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri);
#else
  dart::utils::DartLoader urdfLoader;
  const dart::dynamics::SkeletonPtr skeleton
      = urdfLoader.parseSkeleton(uri, retriever);
#endif

  if (!skeleton)
    throw std::runtime_error("Unable to load '" + uri.toString() + "'");

  if (skeleton->getNumJoints() == 0)
    throw std::runtime_error("Skeleton is empty.");

  if (!transform.matrix().isIdentity())
  {
    auto freeJoint
        = dynamic_cast<dart::dynamics::FreeJoint*>(skeleton->getRootJoint());

    if (!freeJoint)
      throw std::runtime_error(
          "Unable to cast Skeleton's root joint to FreeJoint.");

    freeJoint->setTransform(transform);
  }

  return skeleton;
}

} // namespace io
} // namespace aikido
