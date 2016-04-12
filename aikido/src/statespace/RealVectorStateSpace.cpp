#include <aikido/statespace/RealVectorStateSpace.hpp>

namespace aikido
{
namespace statespace
{
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
Eigen::Map<Eigen::VectorXd> RealVectorStateSpace::getMutableValue(
    State *_state) const
{
  auto valueBuffer =
      reinterpret_cast<double *>(reinterpret_cast<unsigned char *>(_state));

  return Eigen::Map<Eigen::VectorXd>(valueBuffer, mDimension);
}

//=============================================================================
Eigen::Map<const Eigen::VectorXd> RealVectorStateSpace::getValue(
    const State *_state) const
{
  auto valueBuffer = reinterpret_cast<const double *>(
      reinterpret_cast<const unsigned char *>(_state));

  return Eigen::Map<const Eigen::VectorXd>(valueBuffer, mDimension);
}

//=============================================================================
void RealVectorStateSpace::setValue(State *_state,
                                    const Eigen::VectorXd &_value) const
{
  // TODO: Skip this check in release mode.
  if (_value.size() != mDimension) {
    std::stringstream msg;
    msg << "Value has incorrect size: expected " << mDimension << ", got "
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
StateSpace::State *RealVectorStateSpace::allocateStateInBuffer(
    void *_buffer) const
{
  auto state = reinterpret_cast<State *>(_buffer);
  getMutableValue(state).setZero();
  return state;
}

//=============================================================================
void RealVectorStateSpace::freeStateInBuffer(StateSpace::State *_state) const
{
  // Do nothing.
}

//=============================================================================
void RealVectorStateSpace::compose(const StateSpace::State *_state1,
                                   const StateSpace::State *_state2,
                                   StateSpace::State *_out) const
{
  auto state1 = static_cast<const State *>(_state1);
  auto state2 = static_cast<const State *>(_state2);
  auto out = static_cast<State *>(_out);

  setValue(out, getValue(state1) + getValue(state2));
}

//=============================================================================
void RealVectorStateSpace::getIdentity(StateSpace::State *_out) const
{
  auto out = static_cast<State *>(_out);
  setValue(out, Eigen::VectorXd::Zero(mDimension));
}

//=============================================================================
unsigned int RealVectorStateSpace::getDimension() const { return mDimension; }

//=============================================================================
double RealVectorStateSpace::getMaximumExtent() const
{
  return std::numeric_limits<double>::infinity();
}

//=============================================================================
double RealVectorStateSpace::getMeasure() const
{
  return std::numeric_limits<double>::infinity();
}

//=============================================================================
void RealVectorStateSpace::copyState(StateSpace::State *_destination,
                                     const StateSpace::State *_source) const
{
  auto destination = static_cast<State *>(_destination);
  auto source = static_cast<const State *>(_source);
  setValue(destination, getValue(source));
}

//=============================================================================
double RealVectorStateSpace::distance(const StateSpace::State *_state1,
                                      const StateSpace::State *_state2) const
{
  auto v1 = getValue(static_cast<const State *>(_state1));
  auto v2 = getValue(static_cast<const State *>(_state2));
  return (v2 - v1).norm();
}

//=============================================================================
bool RealVectorStateSpace::equalStates(const StateSpace::State *_state1,
                                       const StateSpace::State *_state2) const
{
  return distance(_state1, _state2) < std::numeric_limits<double>::epsilon();
}

//=============================================================================
void RealVectorStateSpace::interpolate(const StateSpace::State *_from,
                                       const StateSpace::State *_to,
                                       const double _alpha,
                                       StateSpace::State *_state) const
{
  if (_alpha < 0. || _alpha > 1.) {
    throw std::invalid_argument("_alpha must be between 0 and 1");
  }

  auto vfrom = getValue(static_cast<const State *>(_from));
  auto vto = getValue(static_cast<const State *>(_to));
  auto vstate = getMutableValue(static_cast<State *>(_state));
  vstate = (1 - _alpha) * vfrom + _alpha * vto;
}

//=============================================================================
void RealVectorStateSpace::expMap(const Eigen::VectorXd &_tangent,
                                  StateSpace::State *_out) const
{
  // TODO: Skip this check in release mode.
  if (_tangent.size() != mDimension) {
    std::stringstream msg;
    msg << "Tangent vector has incorrect size: expected " << mDimension
        << ", got " << _tangent.size() << ".";
    throw std::invalid_argument(msg.str());
  }

  auto out = static_cast<State *>(_out);
  setValue(out, _tangent);
}

}  // namespace statespace
}  // namespace aikido
