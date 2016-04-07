#include <aikido/statespace/CompoundStateSpace.hpp>
#include <iostream>

namespace aikido {
namespace statespace {

//=============================================================================
CompoundStateSpace::CompoundStateSpace(
      const std::vector<StateSpacePtr>& _subspaces)
  : mSubspaces(_subspaces)
  , mOffsets(_subspaces.size(), 0u)
  , mSizeInBytes(0u)
{
  if (!mSubspaces.empty())
  {
    mOffsets[0] = 0;

    for (size_t i = 1; i < mSubspaces.size(); ++i)
      mOffsets[i] = mOffsets[i - 1] + mSubspaces[i - 1]->getStateSizeInBytes();

    mSizeInBytes = mOffsets.back() + mSubspaces.back()->getStateSizeInBytes();
  }
}

//=============================================================================
auto CompoundStateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
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
  auto state = reinterpret_cast<State*>(_buffer);

  for (size_t i = 0; i < mSubspaces.size(); ++i)
    mSubspaces[i]->allocateStateInBuffer(getSubState<>(state, i));

  return state;
}

//=============================================================================
void CompoundStateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  auto state = static_cast<State*>(_state);

  for (size_t i = mSubspaces.size(); i > 0; --i)
    mSubspaces[i - 1]->freeStateInBuffer(getSubState<>(state, i - 1));
}

//=============================================================================
void CompoundStateSpace::compose(
  const StateSpace::State* _state1, const StateSpace::State* _state2,
  StateSpace::State* _out) const
{
  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);
  
  for (size_t i = 0; i < mSubspaces.size(); ++i)
  {
    mSubspaces[i]->compose(getSubState<>(state1, i), getSubState<>(state2, i),
      getSubState<>(out, i));
  }
}

//=============================================================================
unsigned int CompoundStateSpace::getDimension() const 
{
    unsigned int dim = 0;
    for( auto const &sspace: mSubspaces )
    {
        dim += sspace->getDimension();
    }
    return dim;            
}

//=============================================================================
double CompoundStateSpace::getMaximumExtent() const 
{
    
}

//=============================================================================
double CompoundStateSpace::getMeasure() const 
{
    
}

//=============================================================================
bool CompoundStateSpace::satisfiesBounds(const StateSpace::State* _state) const 
{
    
}

//=============================================================================
void CompoundStateSpace::copyState(StateSpace::State* _destination,
                              const StateSpace::State* _source) const
{
    
}

//=============================================================================
double CompoundStateSpace::distance(const StateSpace::State* _state1,
                               const StateSpace::State* _state2) const
{

}

//=============================================================================
bool CompoundStateSpace::equalStates(const StateSpace::State* _state1,
                                const StateSpace::State* _state2) const
{
    
}

//=============================================================================
void CompoundStateSpace::interpolate(const StateSpace::State* _from,
                                const StateSpace::State* _to,
                                const double _t,
                                StateSpace::State* _State) const
{

}

} // namespace statespace
} // namespace aikido
