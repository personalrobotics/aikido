#include <sstream>

namespace aikido {
namespace statespace {

/// \c StateHandle for a \c CompoundStateSpace. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
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

  /// Construct and initialize to \c nullptr.
  CompoundStateHandle()
  {
  }

  /// Construct a handle for \c _state in \c _space.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  CompoundStateHandle(const StateSpace* _space, State* _state)
    : statespace::StateHandle<CompoundStateSpace, QualifiedState>(
        _space, _state)
  {
  }

  /// Gets state by subspace index.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _index in the range [ 0, \c getNumStates() ]
  /// \return state at \c _index
  template <class Space = statespace::StateSpace>
  typename Space::State* getSubState(size_t _index) const
  {
    return this->getStateSpace()->template getSubState<Space>(
      this->getState(), _index);
  }

  /// Gets state by subspace indexa and wraps it in a \c Space::StateHandle
  /// helper class.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _index in the range [ 0, \c getNumStates() ]
  /// \return state at \c _index
  template <class Space = statespace::StateSpace>
  typename Space::StateHandle getSubStateHandle(size_t _index) const
  {
    return typename Space::StateHandle(
      this->getStateSpace()->template getSubSpace<Space>(_index).get(),
      getSubState<Space>(_index));
  }
};

//=============================================================================
template <class Space>
std::shared_ptr<Space> CompoundStateSpace::getSubSpace(size_t _index) const
{
  // TODO: Replace this with a static_cast in release mode.
  const auto rawSpace = mSubspaces[_index];
  auto space = std::dynamic_pointer_cast<Space>(rawSpace);
  if (!space)
  {
    // Create a reference to *rawSpace so we can use it in typeid below. Doing
    // this inline trips -Wpotentially-evaluated-expression in Clang.
    const auto& rawSpaceValue = *rawSpace;

    std::stringstream ss;
    ss << "Requested StateSpace of type '" << typeid(Space).name()
       << "', but the StateSpace at index " << _index
       << " is of incompatible type '" << typeid(rawSpaceValue).name() << "'.";
    throw std::runtime_error(ss.str());
  }
  return space;
}

//=============================================================================
template <class Space>
typename Space::State* CompoundStateSpace::getSubState(
  State* _state, size_t _index) const
{
  // Use getStateSpace() to perform a type-check on the StateSpace.
  getSubSpace(_index);

  return reinterpret_cast<typename Space::State*>(
    reinterpret_cast<char*>(_state) + mOffsets[_index]);
}

//=============================================================================
template <class Space>
const typename Space::State* CompoundStateSpace::getSubState(
  const State* _state, size_t _index) const
{
  // Use getStateSpace() to perform a type-check on the StateSpace.
  getSubSpace(_index);

  return reinterpret_cast<const typename Space::State*>(
    reinterpret_cast<const char*>(_state) + mOffsets[_index]);
}

//=============================================================================
template <class Space>
typename Space::StateHandle CompoundStateSpace::getSubStateHandle(
  State* _state, size_t _index) const
{
  return typename Space::StateHandle(
    getSubSpace<Space>(_index).get(), getSubState<Space>(_state, _index));
}

//=============================================================================
template <class Space>
typename Space::StateHandleConst CompoundStateSpace::getSubStateHandle(
  const State* _state, size_t _index) const
{
  return typename Space::StateHandleConst(
    getSubSpace<Space>(_index).get(), getSubState<Space>(_state, _index));
}

} // namespace statespace
} // namespace aikido
