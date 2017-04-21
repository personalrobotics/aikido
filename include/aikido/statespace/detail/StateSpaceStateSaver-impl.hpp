namespace aikido {
namespace statespace {

StateSpaceStateSaver::StateSpaceStateSaver(MetaSkeletonStateSpacePtr _space)
: mSpace(std::move(_space))
, mPositions(mSpace->getMetaSkeleton()->getPositions())
{
}

StateSpaceStateSaver::~StateSpaceStateSaver()
{
  mSpace->getMetaSkeleton()->setPositions(mPositions);
}

} // namespace statespace
} // namespace aikido
