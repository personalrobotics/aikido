#include <iostream>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
MetaSkeletonStateSaver::MetaSkeletonStateSaver(
    ::dart::dynamics::MetaSkeletonPtr _metaskeleton)
  : mMetaSkeleton(std::move(_metaskeleton))
  , mPositions(mMetaSkeleton->getPositions())
  , mPositionLowerLimits(mMetaSkeleton->getPositionLowerLimits())
  , mPositionUpperLimits(mMetaSkeleton->getPositionUpperLimits())
{
  // Do nothing
}

//==============================================================================
MetaSkeletonStateSaver::~MetaSkeletonStateSaver()
{
  if (static_cast<std::size_t>(mPositions.size())
      != mMetaSkeleton->getNumDofs())
  {
    std::cerr << "[MetaSkeletonStateSaver] The number of DOFs in the "
              << "MetaSkeleton does not match the saved position.";
  }
  if (static_cast<std::size_t>(mPositionLowerLimits.size())
      != mMetaSkeleton->getNumDofs())
  {
    std::cerr << "[MetaSkeletonStateSaver] The number of DOFs in the "
              << "MetaSkeleton does not match the saved joint lower limits.";
  }
  if (static_cast<std::size_t>(mPositionUpperLimits.size())
      != mMetaSkeleton->getNumDofs())
  {
    std::cerr << "[MetaSkeletonStateSaver] The number of DOFs in the "
              << "MetaSkeleton does not match the saved joint upper limits.";
  }

  mMetaSkeleton->setPositions(mPositions);
  mMetaSkeleton->setPositionLowerLimits(mPositionLowerLimits);
  mMetaSkeleton->setPositionUpperLimits(mPositionUpperLimits);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
