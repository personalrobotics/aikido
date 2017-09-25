namespace aikido {
namespace statespace {
namespace dart {

MetaSkeletonStateSpaceSaver::MetaSkeletonStateSpaceSaver(
    MetaSkeletonStateSpacePtr _space)
  : mSpace(std::move(_space))
  , mPositions(mSpace->getMetaSkeleton()->getPositions())
  , mPositionLowerLimits(mSpace->getMetaSkeleton()->getPositionLowerLimits())
  , mPositionUpperLimits(mSpace->getMetaSkeleton()->getPositionUpperLimits())
{
}

MetaSkeletonStateSpaceSaver::~MetaSkeletonStateSpaceSaver()
{
  if (mPositions.size() != mSpace->getMetaSkeleton()->getNumDofs())
  {
    std::cerr << "[MetaSkeletonStateSpaceSaver] The number of DOFs in the "
              << "MetaSkeleton does not match the saved state.";
  }
  if (mPositionLowerLimits.size() != mSpace->getMetaSkeleton()->getNumDofs())
  {
    std::cerr << "[MetaSkeletonStateSpaceSaver] The number of DOFs in the "
              << "MetaSkeleton does not match the save joint lower limits.";
  }
  if (mPositionUpperLimits.size() != mSpace->getMetaSkeleton()->getNumDofs())
  {
    std::cerr << "[MetaSkeletonStateSpaceSaver] The number of DOFs in the "
              << "MetaSkeleton does not match the save joint upper limits.";
  }

  mSpace->getMetaSkeleton()->setPositions(mPositions);
  mSpace->getMetaSkeleton()->setPositionLowerLimits(mPositionLowerLimits);
  mSpace->getMetaSkeleton()->setPositionUpperLimits(mPositionUpperLimits);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
