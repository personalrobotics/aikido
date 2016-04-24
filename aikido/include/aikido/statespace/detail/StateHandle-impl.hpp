namespace aikido {
namespace statespace {

//=============================================================================
template <class _StateSpace, class _QualifiedState>
StateHandle<_StateSpace, _QualifiedState>::StateHandle()
  : mSpace(nullptr)
  , mState(nullptr)
{
}

//=============================================================================
template <class _StateSpace, class _QualifiedState>
StateHandle<_StateSpace, _QualifiedState>::StateHandle(
      const StateSpace* _space, QualifiedState* _state)
  : mSpace(_space)
  , mState(_state)
{
}

//=============================================================================
template <class _StateSpace, class _QualifiedState>
StateHandle<_StateSpace, _QualifiedState>::operator QualifiedState*() const
{
  return mState;
}

//=============================================================================
template <class _StateSpace, class _QualifiedState>
void StateHandle<_StateSpace, _QualifiedState>::reset()
{
  mSpace = nullptr;
  mState = nullptr;
}

//=============================================================================
template <class _StateSpace, class _QualifiedState>
void StateHandle<_StateSpace, _QualifiedState>::reset(
  const StateSpace* _space, QualifiedState* _state)
{
  mSpace = _space;
  mState = _state;
}

//=============================================================================
template <class _StateSpace, class _QualifiedState>
auto StateHandle<_StateSpace, _QualifiedState>::getState() const
  -> QualifiedState*
{
  return mState;
}

//=============================================================================
template <class _StateSpace, class _QualifiedState>
auto StateHandle<_StateSpace, _QualifiedState>::getStateSpace() const
  -> const StateSpace* 
{
  return mSpace;
}

} // namespace statespace
} // namespace aikido
