#ifndef AIKIDO_COMMON_DETAIL_PARSESKELETON_IMPL_HPP_
#define AIKIDO_COMMON_DETAIL_PARSESKELETON_IMPL_HPP_

#include <dart/utils/urdf/DartLoader.hpp>

#include "aikido/common/parseSkeleton.hpp"

namespace aikido {
namespace common {

//==============================================================================
const dart::dynamics::SkeletonPtr parseSkeleton(
    const dart::common::Uri& uri,
    const dart::common::ResourceRetrieverPtr& retriever)
{
#if DART_VERSION_AT_LEAST(6, 12, 0)
  dart::utils::DartLoader::Options options;
  options.mResourceRetriever = retriever;
  dart::utils::DartLoader urdfLoader(options);
  const dart::dynamics::SkeletonPtr skeleton = urdfLoader.parseSkeleton(uri);
#else
  dart::utils::DartLoader urdfLoader;
  const dart::dynamics::SkeletonPtr skeleton
      = urdfLoader.parseSkeleton(uri, retriever);
#endif

  return skeleton;
}

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_DETAIL_PARSESKELETON_IMPL_HPP_
