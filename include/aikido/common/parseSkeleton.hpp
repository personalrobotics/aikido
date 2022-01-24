#ifndef AIKIDO_COMMON_PARSESKELETON_HPP_
#define AIKIDO_COMMON_PARSESKELETON_HPP_

#include <dart/dart.hpp>

namespace aikido {
namespace common {

const dart::dynamics::SkeletonPtr parseSkeleton(
    const dart::common::ResourceRetrieverPtr& retriever,
    const dart::common::Uri& uri);

} // namespace common
} // namespace aikido

#include "aikido/common/detail/parseSkeleton-impl.hpp"

#endif // AIKIDO_COMMON_PARSESKELETON_HPP_
