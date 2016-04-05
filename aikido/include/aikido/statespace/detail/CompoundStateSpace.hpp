namespace aikido {
namespace statespace {

//=============================================================================
template <class Space>
typename Space::State& CompoundStateSpace::getSubStateOf(
  StateSpace::State& _state, size_t _index)
{
#ifndef NDEBUG
  if (!dynamic_cast<Space*>(mSubspaces[_index].get()))
    throw std::runtime_error("Invalid type.");
#endif

  return static_cast<typename Space::State&>(
    getSubState(_state, _index));
}

//=============================================================================
template <class Space>
const typename Space::State& CompoundStateSpace::getSubStateOf(
  const StateSpace::State& _state, size_t _index)
{
#ifndef NDEBUG
  if (!dynamic_cast<Space*>(mSubspaces[_index].get()))
    throw std::runtime_error("Invalid type.");
#endif

  return static_cast<const typename Space::State&>(
    getSubState(_state, _index));
}

} // namespace statespace
} // namespace aikido
