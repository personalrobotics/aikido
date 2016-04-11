#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpaceSampleableConstraint.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
RealVectorStateSpace::RealVectorStateSpace(int _dimension)
  : mDimension(_dimension)
{
}

//=============================================================================
auto RealVectorStateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
Eigen::Map<Eigen::VectorXd> RealVectorStateSpace::getMutableValue(State* _state) const
{
  auto valueBuffer = reinterpret_cast<double*>(
    reinterpret_cast<unsigned char*>(_state));

  return Eigen::Map<Eigen::VectorXd>(valueBuffer, mDimension);
}

//=============================================================================
Eigen::Map<const Eigen::VectorXd> RealVectorStateSpace::getValue(
  const State* _state) const
{
  auto valueBuffer = reinterpret_cast<const double*>(
    reinterpret_cast<const unsigned char*>(_state));

  return Eigen::Map<const Eigen::VectorXd>(valueBuffer, mDimension);
}

//=============================================================================
int RealVectorStateSpace::getDimension() const
{
  return mDimension;
}

//=============================================================================
void RealVectorStateSpace::setValue(
  State* _state, const Eigen::VectorXd& _value) const
{
  // TODO: Skip this check in release mode.
  if (_value.size() != mDimension)
  {
    std::stringstream msg;
    msg << "Value has incorrect size: expected " << mDimension  << ", got "
        << _value.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  getMutableValue(_state) = _value;
}

//=============================================================================
size_t RealVectorStateSpace::getStateSizeInBytes() const
{
  return mDimension * sizeof(double);
}

//=============================================================================
StateSpace::State* RealVectorStateSpace::allocateStateInBuffer(
  void* _buffer) const
{
  auto state = reinterpret_cast<State*>(_buffer);
  getMutableValue(state).setZero();
  return state;
}

//=============================================================================
void RealVectorStateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  // Do nothing.
}

//=============================================================================
void RealVectorStateSpace::compose(
  const StateSpace::State* _state1, const StateSpace::State* _state2,
  StateSpace::State* _out) const
{
  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  setValue(out, getValue(state1) + getValue(state2));
}

//=============================================================================
void RealVectorStateSpace::expMap(
  const Eigen::VectorXd& _tangent, StateSpace::State* _out) const
{
  // TODO: Skip this check in release mode.
  if (_tangent.size() != mDimension)
  {
    std::stringstream msg;
    msg << "Tangent vector has incorrect size: expected " << mDimension
        << ", got " << _tangent.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  auto out = static_cast<State*>(_out);
  setValue(out, _tangent);
}


} // namespace statespace
} // namespace aikido
