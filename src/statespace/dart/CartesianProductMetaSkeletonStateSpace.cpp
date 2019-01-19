#include "aikido/statespace/dart/CartesianProductMetaSkeletonStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
CartesianProductMetaSkeletonStateSpace::CartesianProductMetaSkeletonStateSpace(
  std::vector<ConstStateSpacePtr> subspaces,
  ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace)
: CartesianProduct(std::move(subspaces))
, mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
{
  // do nothing.
}

//==============================================================================
ConstMetaSkeletonStateSpacePtr CartesianProductMetaSkeletonStateSpace::getMetaSkeletonStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

} // namespace dart
} // namespace statespace
} // namespace aikido
