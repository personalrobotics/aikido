namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
template <class Space>
std::shared_ptr<Space> MetaSkeletonStateSpace::getJointSpace(
    const ::dart::dynamics::Joint* _joint) const
{
  const auto index = mMetaSkeleton->getIndexOf(_joint, true);
  if (index == ::dart::dynamics::INVALID_INDEX)
    throw std::invalid_argument("Joint is not in MetaSkeleton.");

  return getSubspace<Space>(index);
}

//==============================================================================
template <class Space>
std::shared_ptr<Space> MetaSkeletonStateSpace::getJointSpace(
    std::size_t _index) const
{
  return getSubspace<Space>(_index);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
