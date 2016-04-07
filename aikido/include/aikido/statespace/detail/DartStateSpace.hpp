namespace aikido {
namespace statespace {

//=============================================================================
template <class Space>
const StateSpace* MetaSkeletonStateSpace::getJointSpace(
  const dart::dynamics::Joint* _joint) const
{
  const auto index = mMetaSkeleton->getIndexOf(_joint, true);
  if (index == dart::dynamics::INVALID_INDEX)
    return nullptr;

  return getSubSpace(index);
}

} // namespace statespace
} // namespace aikido
