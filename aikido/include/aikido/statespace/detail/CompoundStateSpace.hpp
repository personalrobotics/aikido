#include <sstream>

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
  // TODO: Repalce this with a static_cast in release mode.
  const auto& raw_space = mSubspaces[_index].get();
  auto space = dynamic_cast<const Space*>(raw_space);
  if (!space)
  {
    std::stringstream ss;
    ss << "Requested StateSpace of type '" << typeid(Space).name()
       << "', but the StateSpace at index " << _index
       << " is of incompatible type '" << typeid(*raw_space).name() << "'.";
    throw std::runtime_error(ss.str());
  }
  return *space;
}

//=============================================================================
template <class Space>
typename Space::State& CompoundStateSpace::getSubState(
  StateSpace::State& _state, size_t _index) const
{
  // Use getStateSpace() to perform a type-check on the StateSpace.
  getSubSpace(_index);

  return reinterpret_cast<typename Space::State&>(
    *(reinterpret_cast<char*>(&_state) + mOffsets[_index]));
}

//=============================================================================
template <class Space>
const typename Space::State& CompoundStateSpace::getSubState(
  const StateSpace::State& _state, size_t _index) const
{
  // Use getStateSpace() to perform a type-check on the StateSpace.
  getSubSpace(_index);

  return reinterpret_cast<const typename Space::State&>(
    *(reinterpret_cast<const char*>(&_state) + mOffsets[_index]));
}

//=============================================================================
template <class Space>
typename Space::StateHandle CompoundStateSpace::getSubStateHandle(
  StateSpace::State& _state, size_t _index) const
{
  return typename Space::StateHandle(
    &getSubSpace<Space>(_index), &getSubState<Space>(_state, _index));
}

//=============================================================================
template <class Space>
typename Space::StateHandleConst CompoundStateSpace::getSubStateHandle(
  const StateSpace::State& _state, size_t _index) const
{
  return typename Space::StateHandleConst(
    &getSubSpace<Space>(_index), &getSubState<Space>(_state, _index));
}

} // namespace statespace
} // namespace aikido
