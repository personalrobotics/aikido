#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
MetaSkeletonStateSpaceSaver::MetaSkeletonStateSpaceSaver(
    MetaSkeletonStateSpacePtr _space)
  : mSpace(std::move(_space))
  , mPositions(mSpace->getMetaSkeleton()->getPositions())
  , mPositionLowerLimits(mSpace->getMetaSkeleton()->getPositionLowerLimits())
  , mPositionUpperLimits(mSpace->getMetaSkeleton()->getPositionUpperLimits())
{
}

//==============================================================================
MetaSkeletonStateSpaceSaver::~MetaSkeletonStateSpaceSaver()
{
  if (static_cast<std::size_t>(mPositions.size())
      != mSpace->getMetaSkeleton()->getNumDofs())
  {
    std::cerr << "[MetaSkeletonStateSpaceSaver] The number of DOFs in the "
              << "MetaSkeleton does not match the saved position.";
  }
  if (static_cast<std::size_t>(mPositionLowerLimits.size())
      != mSpace->getMetaSkeleton()->getNumDofs())
  {
    std::cerr << "[MetaSkeletonStateSpaceSaver] The number of DOFs in the "
              << "MetaSkeleton does not match the saved joint lower limits.";
  }
  if (static_cast<std::size_t>(mPositionUpperLimits.size())
      != mSpace->getMetaSkeleton()->getNumDofs())
  {
    std::cerr << "[MetaSkeletonStateSpaceSaver] The number of DOFs in the "
              << "MetaSkeleton does not match the saved joint upper limits.";
  }

  mSpace->getMetaSkeleton()->setPositions(mPositions);
  mSpace->getMetaSkeleton()->setPositionLowerLimits(mPositionLowerLimits);
  mSpace->getMetaSkeleton()->setPositionUpperLimits(mPositionUpperLimits);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
