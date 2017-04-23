namespace aikido {
namespace statespace {
namespace dart {

MetaSkeletonStateSpaceSaver::MetaSkeletonStateSpaceSaver(MetaSkeletonStateSpacePtr _space)
: mSpace(std::move(_space))
, mPositions(mSpace->getMetaSkeleton()->getPositions())
{
}

MetaSkeletonStateSpaceSaver::~MetaSkeletonStateSpaceSaver()
{
  if (mPositions.size() != mSpace->getMetaSkeleton().getNumDofs())
  {
    dtwarn << "[MetaSkeletonStateSpaceSaver] The number of DOFs in the "
           << "MetaSkeleton does not match the saved state.";
  }
  mSpace->getMetaSkeleton()->setPositions(mPositions);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
