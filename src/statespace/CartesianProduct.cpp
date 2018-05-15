#include <iostream>
#include <aikido/statespace/CartesianProduct.hpp>

namespace aikido {
namespace statespace {

//==============================================================================
CartesianProduct::CartesianProduct(std::vector<StateSpacePtr> _subspaces)
  : mSubspaces(std::move(_subspaces))
  , mOffsets(mSubspaces.size(), 0u)
  , mSizeInBytes(0u)
{
  for (const auto& subspace : mSubspaces)
  {
    if (subspace == nullptr)
      throw std::invalid_argument("Subspace is null.");
  }

  if (!mSubspaces.empty())
  {
    for (std::size_t i = 1; i < mSubspaces.size(); ++i)
      mOffsets[i] = mOffsets[i - 1] + mSubspaces[i - 1]->getStateSizeInBytes();

    mSizeInBytes = mOffsets.back() + mSubspaces.back()->getStateSizeInBytes();
  }
}

//==============================================================================
auto CartesianProduct::createState() const -> ScopedState
{
  return ScopedState(this);
}

//==============================================================================
CartesianProduct::ScopedState CartesianProduct::cloneState(
    const StateSpace::State* stateIn) const
{
  auto newState = createState();
  copyState(stateIn, newState);

  return newState;
}

//==============================================================================
std::size_t CartesianProduct::getNumSubspaces() const
{
  return mSubspaces.size();
}

//==============================================================================
std::size_t CartesianProduct::getStateSizeInBytes() const
{
  return mSizeInBytes;
}

//==============================================================================
StateSpace::State* CartesianProduct::allocateStateInBuffer(void* _buffer) const
{
  auto state = reinterpret_cast<State*>(_buffer);

  for (std::size_t i = 0; i < mSubspaces.size(); ++i)
    mSubspaces[i]->allocateStateInBuffer(getSubState<>(state, i));

  return state;
}

//==============================================================================
void CartesianProduct::freeStateInBuffer(StateSpace::State* _state) const
{
  auto state = static_cast<State*>(_state);

  for (std::size_t i = mSubspaces.size(); i > 0; --i)
    mSubspaces[i - 1]->freeStateInBuffer(getSubState<>(state, i - 1));
}

//==============================================================================
void CartesianProduct::compose(
    const StateSpace::State* _state1,
    const StateSpace::State* _state2,
    StateSpace::State* _out) const
{
  // TODO: Disable this in release mode.
  if (_state1 == _out || _state2 == _out)
    throw std::invalid_argument("Output aliases input.");

  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  for (std::size_t i = 0; i < mSubspaces.size(); ++i)
  {
    mSubspaces[i]->compose(
        getSubState<>(state1, i),
        getSubState<>(state2, i),
        getSubState<>(out, i));
  }
}

//==============================================================================
void CartesianProduct::getIdentity(StateSpace::State* _out) const
{
  auto state = static_cast<State*>(_out);

  for (std::size_t i = 0; i < mSubspaces.size(); ++i)
  {
    mSubspaces[i]->getIdentity(getSubState<>(state, i));
  }
}

//==============================================================================
void CartesianProduct::getInverse(
    const StateSpace::State* _in, StateSpace::State* _out) const
{
  // TODO: Disable this in release mode.
  if (_out == _in)
    throw std::invalid_argument("Output aliases input.");

  auto in = static_cast<const State*>(_in);
  auto out = static_cast<State*>(_out);

  for (std::size_t i = 0; i < mSubspaces.size(); ++i)
  {
    mSubspaces[i]->getInverse(getSubState<>(in, i), getSubState<>(out, i));
  }
}

//==============================================================================
std::size_t CartesianProduct::getDimension() const
{
  std::size_t dim = 0;
  for (auto const& sspace : mSubspaces)
  {
    dim += sspace->getDimension();
  }
  return dim;
}

//==============================================================================
void CartesianProduct::copyState(
    const StateSpace::State* _source, StateSpace::State* _destination) const
{
  auto destination = static_cast<State*>(_destination);
  auto source = static_cast<const State*>(_source);
  for (std::size_t i = 0; i < mSubspaces.size(); ++i)
  {
    mSubspaces[i]->copyState(
        getSubState<>(source, i), getSubState<>(destination, i));
  }
}

//==============================================================================
void CartesianProduct::expMap(
    const Eigen::VectorXd& _tangent, StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);
  auto dimension = getDimension();

  // TODO: Skip these checks in release mode.
  if (static_cast<std::size_t>(_tangent.rows()) != dimension)
  {
    std::stringstream msg;
    msg << "_tangent has incorrect size: expected " << dimension << ", got "
        << _tangent.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  int index = 0;
  for (std::size_t i = 0; i < mSubspaces.size(); ++i)
  {
    auto dim = mSubspaces[i]->getDimension();
    mSubspaces[i]->expMap(
        _tangent.block(index, 0, dim, 1), getSubState<>(out, i));
    index += dim;
  }
}

//==============================================================================
void CartesianProduct::logMap(
    const StateSpace::State* _in, Eigen::VectorXd& _tangent) const
{
  auto dimension = getDimension();

  if (static_cast<std::size_t>(_tangent.rows()) != dimension)
  {
    _tangent.resize(dimension);
  }

  auto in = static_cast<const State*>(_in);

  int index = 0;
  for (std::size_t i = 0; i < mSubspaces.size(); ++i)
  {
    auto dim = mSubspaces[i]->getDimension();
    Eigen::VectorXd segment(dim);
    mSubspaces[i]->logMap(getSubState<>(in, i), segment);

    _tangent.segment(index, dim) = segment;

    index += dim;
  }
}

//==============================================================================
void CartesianProduct::print(
    const StateSpace::State* _state, std::ostream& _os) const
{

  auto state = static_cast<const State*>(_state);
  for (std::size_t i = 0; i < mSubspaces.size(); ++i)
  {
    _os << "[ " << i << ":";
    getSubspace<>(i)->print(getSubState<>(state, i), _os);
    _os << " ] ";
  }
}

} // namespace statespace
} // namespace aikido
