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
  mSpace->getMetaSkeleton()->setPositions(mPositions);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
