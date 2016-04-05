namespace aikido {
namespace statespace {

//=============================================================================
template <class Space>
const Space& CompoundStateSpace::getSubSpace(size_t _index) const
{
  auto space = dynamic_cast<const Space*>(mSubspaces[_index].get());
  if (!space)
    throw std::runtime_error("Invalid type.");

  return *space;
}

//=============================================================================
template <class Space>
typename Space::State& CompoundStateSpace::getSubState(
  StateSpace::State& _state, size_t _index) const
{
  // TODO: Only in debug mode.
  if (!dynamic_cast<Space*>(mSubspaces[_index].get()))
    throw std::runtime_error("Invalid type.");

  return reinterpret_cast<typename Space::State&>(
    *(reinterpret_cast<char*>(&_state) + mOffsets[_index]));
}

//=============================================================================
template <class Space>
const typename Space::State& CompoundStateSpace::getSubState(
  const StateSpace::State& _state, size_t _index) const
{
  // TODO: Only in debug mode.
  if (!dynamic_cast<Space*>(mSubspaces[_index].get()))
    throw std::runtime_error("Invalid type.");

  return reinterpret_cast<const typename Space::State&>(
    *(reinterpret_cast<const char*>(&_state) + mOffsets[_index]));
}

//=============================================================================
template <class Space>
typename Space::StateHandle CompoundStateSpace::getSubStateHandle(
  const StateSpace::State& _state, size_t _index) const
{
  return Space::StateHandle(this, getSubState<Space>(_state, _index));
}

} // namespace statespace
} // namespace aikido
