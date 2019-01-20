#ifndef AIKIDO_STATESPACE_DART_CARTESIANPRODUCTMETASKELETONSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_CARTESIANPRODUCTMETASKELETONSTATESPACE_HPP_

#include "aikido/statespace/CartesianProduct.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

AIKIDO_DECLARE_POINTERS(CartesianProductMetaSkeletonStateSpace)

/// \c StateSpace of a DART \c MetaSkeleton with artificially created state
/// space.
/// It is a CartesianProduct but has access to the original
/// MetaSkeletonStateSpace.
/// This class is necessary when a different state space from
/// MetaSkeletonStateSpace
/// needs to be used, e.g. for the control of SO2 joint.
class CartesianProductMetaSkeletonStateSpace : public CartesianProduct
{
public:
  /// Constructor.
  /// \param[in] subspaces Vector of subspaces
  /// \param[in] metaSkeletonStateSpace
  explicit CartesianProductMetaSkeletonStateSpace(
      std::vector<ConstStateSpacePtr> subspaces,
      ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace);

  /// \return MetaSkeletonStateSpace associated with this statespace.
  ConstMetaSkeletonStateSpacePtr getMetaSkeletonStateSpace() const;

private:
  ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif
