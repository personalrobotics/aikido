#include <aikido/statespace/CompoundStateSpace.hpp>
#include <iostream>

namespace aikido {
namespace statespace {

//=============================================================================
CompoundStateSpace::CompoundStateSpace(
      const std::vector<StateSpacePtr>& _subspaces)
  : mSubspaces(_subspaces)
  , mOffsets(_subspaces.size(), 0u)
{
  mOffsets[0] = sizeof(State);

  for (size_t i = 1; i < mSubspaces.size(); ++i)
    mOffsets[i] = mOffsets[i - 1] + mSubspaces[i - 1]->getStateSizeInBytes();

  if (!mSubspaces.empty())
    mSizeInBytes = mOffsets.back() + mSubspaces.back()->getStateSizeInBytes();
  else
    mSizeInBytes = 0u;
}

//=============================================================================
size_t CompoundStateSpace::getNumStates() const
{
  return mSubspaces.size();
}

//=============================================================================
size_t CompoundStateSpace::getStateSizeInBytes() const
{
  return mSizeInBytes;
}

//=============================================================================
StateSpace::State* CompoundStateSpace::allocateStateInBuffer(
  void* _buffer) const
{
  State* const state = new (_buffer) State;

  for (size_t i = 0; i < mSubspaces.size(); ++i)
    mSubspaces[i]->allocateStateInBuffer(&getSubState<>(*state, i));

  return state;
}

//=============================================================================
void CompoundStateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  auto state = static_cast<State*>(_state);

  for (size_t i = mSubspaces.size(); i > 0; --i)
    mSubspaces[i - 1]->freeStateInBuffer(&getSubState<>(*state, i - 1));

  reinterpret_cast<State*>(_state)->~State();
}

//=============================================================================
void CompoundStateSpace::compose(
  const StateSpace::State& _state1, const StateSpace::State& _state2,
  StateSpace::State& _out) const
{
  const auto& state1 = static_cast<const State&>(_state1);
  const auto& state2 = static_cast<const State&>(_state2);
  auto& out = static_cast<State&>(_out);
  
  for (size_t i = 0; i < mSubspaces.size(); ++i)
  {
    mSubspaces[i]->compose(getSubState<>(state1, i), getSubState<>(state2, i),
      getSubState<>(out, i));
  }
}

} // namespace statespace
} // namespace aikido
