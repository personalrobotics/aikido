namespace aikido {
namespace statespace {

template <class _QualifiedState>
class CompoundStateHandle 
  : public statespace::StateHandle<CompoundStateSpace, _QualifiedState>
{
public:
  using typename statespace::StateHandle<
    CompoundStateSpace, _QualifiedState>::State;
  using typename statespace::StateHandle<
    CompoundStateSpace, _QualifiedState>::StateSpace;
  using typename statespace::StateHandle<
    CompoundStateSpace, _QualifiedState>::QualifiedState;


  CompoundStateHandle()
  {
  }

  CompoundStateHandle(const StateSpace* _space, State* _state)
    : statespace::StateHandle<CompoundStateSpace, QualifiedState>(
        _space, _state)
  {
  }

  /// Gets state of type by subspace index.
  template <class Space = statespace::StateSpace>
  typename Space::State& getSubState(size_t _index) const
  {
    return this->getStateSpace()->template getSubState<Space>(
      *this->getState(), _index);
  }

  /// Gets state of type by subspace index.
  template <class Space = statespace::StateSpace>
  typename Space::StateHandle getSubStateHandle(size_t _index) const
  {
    return typename Space::StateHandle(
      &this->getStateSpace()->template getSubSpace<Space>(_index),
      &getSubState<Space>(_index));
  }
};

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

} // namespace statespace
} // namespace aikido
