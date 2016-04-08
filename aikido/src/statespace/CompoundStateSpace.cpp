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
void CompoundStateSpace::expMap(
  const Eigen::VectorXd& _tangent, StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);
  
  int dimension = getDimension();

  // TODO: Skip these checks in release mode.
  if (_tangent.rows() != dimension)
  {
    std::stringstream msg;
    msg << "_tangent has incorrect size: expected " << dimension
        << ", got " << _tangent.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  int index = 0;
  for (size_t i = 0; i < mSubspaces.size(); ++i)
  {
    int dim = mSubspaces[i]->getDimension();
    mSubspaces[i]->expMap(
      _tangent.block(index, 0, dim, 1),
      getSubState<>(out, i));
    index += dim;
  }
}

//=============================================================================
int CompoundStateSpace::getDimension() const
{
  int dim = 0; 

  for (size_t i = 0; i < mSubspaces.size(); ++i)
  {
    dim += mSubspaces[i]->getDimension();
  }

  return dim;
}

} // namespace statespace
} // namespace aikido
