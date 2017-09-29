#include <aikido/statespace/SO2.hpp>

namespace aikido {
namespace statespace {
//==============================================================================
SO2::State::State() : mAngle(0.)
{
}

//==============================================================================
SO2::State::State(double _angle) : mAngle(_angle)
{
}

//==============================================================================
double SO2::State::getAngle() const
{
  return mAngle;
}

//==============================================================================
void SO2::State::setAngle(double _angle)
{
  mAngle = _angle;
}

//==============================================================================
Eigen::Rotation2Dd SO2::State::getRotation() const
{
  return Eigen::Rotation2Dd(mAngle);
}

//==============================================================================
void SO2::State::setRotation(const Eigen::Rotation2Dd& _rotation)
{
  mAngle = _rotation.angle();
}

//==============================================================================
auto SO2::createState() const -> ScopedState
{
  return ScopedState(this);
}

//==============================================================================
double SO2::getAngle(const State* _state) const
{
  return _state->getAngle();
}

//==============================================================================
void SO2::setAngle(State* _state, double _angle) const
{
  _state->setAngle(_angle);
}

//==============================================================================
Eigen::Rotation2Dd SO2::getRotation(const State* _state) const
{
  return Eigen::Rotation2Dd(_state->mAngle);
}

//==============================================================================
void SO2::setRotation(State* _state, const Eigen::Rotation2Dd& _rotation) const
{
  _state->mAngle = _rotation.angle();
}

//==============================================================================
size_t SO2::getStateSizeInBytes() const
{
  return sizeof(State);
}

//==============================================================================
StateSpace::State* SO2::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//==============================================================================
void SO2::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//==============================================================================
void SO2::compose(
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

  out->mAngle = state1->mAngle + state2->mAngle;
}

//==============================================================================
void SO2::getIdentity(StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);
  setAngle(out, 0.);
}

//==============================================================================
void SO2::getInverse(
    const StateSpace::State* _in, StateSpace::State* _out) const
{
  // TODO: Disable this in release mode.
  if (_out == _in)
    throw std::invalid_argument("Output aliases input.");

  auto in = static_cast<const State*>(_in);
  auto out = static_cast<State*>(_out);

  setAngle(out, -getAngle(in));
}

//==============================================================================
size_t SO2::getDimension() const
{
  return 1;
}

//==============================================================================
void SO2::copyState(
    const StateSpace::State* _source, StateSpace::State* _destination) const
{
  auto source = static_cast<const State*>(_source);
  auto destination = static_cast<State*>(_destination);
  setAngle(destination, getAngle(source));
}

//==============================================================================
void SO2::expMap(const Eigen::VectorXd& _tangent, StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);

  // TODO: Skip these checks in release mode.
  if (_tangent.rows() != 1)
  {
    std::stringstream msg;
    msg << "_tangent has incorrect size: expected 1"
        << ", got " << _tangent.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  double angle = _tangent(0);
  out->mAngle = angle;
}

//==============================================================================
void SO2::logMap(const StateSpace::State* _in, Eigen::VectorXd& _tangent) const
{
  if (_tangent.rows() != 1)
  {
    _tangent.resize(1);
  }

  auto in = static_cast<const State*>(_in);
  _tangent(0) = getAngle(in);
}

//==============================================================================
void SO2::print(const StateSpace::State* _state, std::ostream& _os) const
{
  auto state = static_cast<const State*>(_state);
  _os << "[" << getAngle(state) << "]";
}

} // namespace statespace
} // namespace aikido
