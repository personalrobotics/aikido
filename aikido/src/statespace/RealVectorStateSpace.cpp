#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <boost/format.hpp>

using boost::format;
using boost::str;

namespace aikido {
namespace statespace {

//=============================================================================
RealVectorStateSpace::State::State(const Eigen::VectorXd& _x)
  : mValue(_x)
{
}

//=============================================================================
const Eigen::VectorXd& RealVectorStateSpace::State::getValue() const
{
  return mValue;
}

//=============================================================================
void RealVectorStateSpace::State::setValue(const Eigen::VectorXd& _x)
{
  mValue = _x;
}

//=============================================================================
RealVectorStateSpace::RealVectorStateSpace(int _dimension)
  : mDimension(_dimension)
{
  if (mDimension < 0)
    throw std::invalid_argument("_dimension must be positive.");
}

//=============================================================================
size_t RealVectorStateSpace::getStateSizeInBytes() const
{
  return sizeof(State);
}

//=============================================================================
StateSpace::State* RealVectorStateSpace::allocateState() const
{
  return new State(Eigen::VectorXd::Zero(mDimension));
}

//=============================================================================
StateSpace::State* RealVectorStateSpace::allocateStateInBuffer(
  void* _buffer) const
{
  // TODO: Allocate this in contiguous memory.
  return new (_buffer) State(Eigen::VectorXd::Zero(mDimension));
}

//=============================================================================
void RealVectorStateSpace::freeState(StateSpace::State* _state) const
{
  // TODO: Allocate this in contiguous memory.
  delete static_cast<State *>(_state);
}

//=============================================================================
void RealVectorStateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//=============================================================================
void RealVectorStateSpace::compose(
  const StateSpace::State& _state1, const StateSpace::State& _state2,
  StateSpace::State& _out) const
{
  const auto& state1 = static_cast<const State&>(_state1);
  const auto& state2 = static_cast<const State&>(_state2);
  
  if (state1.mValue.rows() != state2.mValue.rows())
    throw std::invalid_argument(
      "_state1 and state2 must have same dimension.");

  auto& out = static_cast<State&>(_out);
  out.mValue = state1.mValue + state2.mValue;
}

} // namespace statespace
} // namespace aikido
