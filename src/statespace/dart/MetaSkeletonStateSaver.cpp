#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"

#include <iostream>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
MetaSkeletonStateSaver::MetaSkeletonStateSaver(
    ::dart::dynamics::MetaSkeletonPtr metaskeleton, int options)
  : mMetaSkeleton(std::move(metaskeleton)), mOptions(std::move(options))
{
  if (mOptions & Options::POSITIONS)
    mPositions = mMetaSkeleton->getPositions();

  if (mOptions & Options::POSITION_LIMITS)
  {
    mPositionLowerLimits = mMetaSkeleton->getPositionLowerLimits();
    mPositionUpperLimits = mMetaSkeleton->getPositionUpperLimits();
  }
}

//==============================================================================
MetaSkeletonStateSaver::~MetaSkeletonStateSaver()
{
  if (mOptions & Options::POSITIONS)
  {
    if (static_cast<std::size_t>(mPositions.size())
        != mMetaSkeleton->getNumDofs())
    {
      std::cerr << "[MetaSkeletonStateSaver] The number of DOFs in the "
                << "MetaSkeleton does not match the saved position.";
    }

    mMetaSkeleton->setPositions(mPositions);
  }

  if (mOptions & Options::POSITION_LIMITS)
  {
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

    mMetaSkeleton->setPositionLowerLimits(mPositionLowerLimits);
    mMetaSkeleton->setPositionUpperLimits(mPositionUpperLimits);
  }
}

} // namespace dart
} // namespace statespace
} // namespace aikido
