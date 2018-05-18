#include "aikido/statespace/CartesianProduct.hpp"

#include <sstream>

namespace aikido {
namespace statespace {

/// \c StateHandle for a \c CartesianProduct. The template parameter is
/// necessary to support both \c const and non-<tt>const</tt> states.
template <class _QualifiedState>
class CompoundStateHandle
    : public statespace::StateHandle<CartesianProduct, _QualifiedState>
{
public:
  using typename statespace::StateHandle<CartesianProduct,
                                         _QualifiedState>::StateSpace;

  using typename statespace::StateHandle<CartesianProduct,
                                         _QualifiedState>::QualifiedState;

  using typename statespace::StateHandle<CartesianProduct,
                                         _QualifiedState>::State;
  using typename statespace::StateHandle<CartesianProduct,
                                         _QualifiedState>::ConstState;

  using NonConstHandle = CompoundStateHandle<State>;
  using ConstHandle = CompoundStateHandle<ConstState>;

  /// Construct and initialize to \c nullptr.
  CompoundStateHandle()
  {
    // Do nothing
  }

  /// Construct a handle for \c _state in \c _space.
  ///
  /// \param _space state space that created \c _state
  /// \param _state state created by \c _space
  CompoundStateHandle(const StateSpace* _space, State* _state)
    : statespace::StateHandle<CartesianProduct, QualifiedState>(_space, _state)
  {
    // Do nothing
  }

  /// Gets state by subspace index.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _index in the range [ 0, \c getNumSubspaces() ]
  /// \return state at \c _index
  template <class Space = statespace::StateSpace>
  typename Space::State* getSubState(std::size_t _index)
  {
    return this->getStateSpace()->template getSubState<Space>(
        this->getState(), _index);
  }

  /// Gets state by subspace index.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _index in the range [ 0, \c getNumSubspaces() ]
  /// \return state at \c _index
  template <class Space = statespace::StateSpace>
  const typename Space::State* getSubState(std::size_t _index) const
  {
    return this->getStateSpace()->template getSubState<Space>(
        this->getState(), _index);
  }

  /// Gets state by subspace index and wraps it in a \c Space::StateHandle
  /// helper class.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _index in the range [ 0, \c getNumSubspaces() ]
  /// \return state at \c _index
  template <class Space = statespace::StateSpace>
  typename Space::StateHandle getSubStateHandle(std::size_t _index)
  {
    return typename Space::StateHandle(
        this->getStateSpace()->template getSubspace<Space>(_index).get(),
        getSubState<Space>(_index));
  }

  /// Gets state by subspace index and wraps it in a \c Space::StateHandle
  /// helper class.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _index in the range [ 0, \c getNumSubspaces() ]
  /// \return state at \c _index
  template <class Space = statespace::StateSpace>
  typename Space::StateHandleConst getSubStateHandle(std::size_t _index) const
  {
    return typename Space::StateHandleConst(
        this->getStateSpace()->template getSubspace<Space>(_index).get(),
        getSubState<Space>(_index));
  }
};

//==============================================================================
template <class Space>
std::shared_ptr<const Space> CartesianProduct::getSubspace(
    std::size_t _index) const
{
  // TODO: Replace this with a static_cast in release mode.
  const auto rawSpace = mSubspaces[_index];
  auto space = std::dynamic_pointer_cast<const Space>(rawSpace);
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

//==============================================================================
template <class Space>
typename Space::State* CartesianProduct::getSubState(
    State* _state, std::size_t _index) const
{
  // Use getStateSpace() to perform a type-check on the StateSpace.
  getSubspace(_index);

  return reinterpret_cast<typename Space::State*>(
      reinterpret_cast<char*>(_state) + mOffsets[_index]);
}

//==============================================================================
template <class Space>
const typename Space::State* CartesianProduct::getSubState(
    const State* _state, std::size_t _index) const
{
  // Use getStateSpace() to perform a type-check on the StateSpace.
  getSubspace(_index);

  return reinterpret_cast<const typename Space::State*>(
      reinterpret_cast<const char*>(_state) + mOffsets[_index]);
}

//==============================================================================
template <class Space>
typename Space::StateHandle CartesianProduct::getSubStateHandle(
    State* _state, std::size_t _index) const
{
  return typename Space::StateHandle(
      getSubspace<Space>(_index).get(), getSubState<Space>(_state, _index));
}

//==============================================================================
template <class Space>
typename Space::StateHandleConst CartesianProduct::getSubStateHandle(
    const State* _state, std::size_t _index) const
{
  return typename Space::StateHandleConst(
      getSubspace<Space>(_index).get(), getSubState<Space>(_state, _index));
}

} // namespace statespace
} // namespace aikido
