#include <aikido/statespace/CompoundStateSpace.hpp>
#include <iostream>

namespace aikido {
namespace statespace {

//=============================================================================
CompoundStateSpace::State::State(
      const std::vector<StateSpace::State*>& _states)
  : mValue(_states)
{
}

//=============================================================================
size_t CompoundStateSpace::State::getNumStates() const
{
  return mValue.size();
}

//=============================================================================
StateSpace::State& CompoundStateSpace::State::getState(size_t _index)
{
  return *mValue[_index];
}

//=============================================================================
const StateSpace::State& CompoundStateSpace::State::getState(
  size_t _index) const
{
  return *mValue[_index];
}

//=============================================================================
const std::vector<StateSpace::State*>& CompoundStateSpace::State::getStates()
{
  return mValue;
}

//=============================================================================
std::vector<const StateSpace::State*>
  CompoundStateSpace::State::getStates() const
{
  std::vector<const StateSpace::State*> output;
  output.reserve(mValue.size());

  for (StateSpace::State* state : mValue)
    output.push_back(state);

  return output;
}

//=============================================================================
CompoundStateSpace::CompoundStateSpace(
      const std::vector<StateSpacePtr>& _subspaces)
  : mSubspaces(_subspaces)
{
}

//=============================================================================
StateSpace::State* CompoundStateSpace::allocateState() const
{
  std::vector<StateSpace::State*> substates;
  substates.reserve(mSubspaces.size());

  for (const StateSpacePtr& stateSpace : mSubspaces)
    substates.push_back(stateSpace->allocateState());

  return new State(substates);
}

//=============================================================================
void CompoundStateSpace::freeState(StateSpace::State* _state) const
{
  auto compound_state = static_cast<State *>(_state);

  for (size_t i = 0; i < mSubspaces.size(); ++i)
    mSubspaces[i]->freeState(&compound_state->getState(i));

  delete compound_state;
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
    mSubspaces[i]->compose(
      *state1.mValue[i], *state2.mValue[i], *out.mValue[i]);
  }
}

} // namespace statespace
} // namespace aikido
