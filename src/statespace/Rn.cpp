#include <aikido/statespace/Rn.hpp>

namespace aikido
{
namespace statespace
{
//=============================================================================
Rn::Rn(int _dimension)
    : mDimension(_dimension)
{
}

//=============================================================================
auto Rn::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
Eigen::Map<Eigen::VectorXd> Rn::getMutableValue(
    State *_state) const
{
  auto valueBuffer =
      reinterpret_cast<double *>(reinterpret_cast<unsigned char *>(_state));

  return Eigen::Map<Eigen::VectorXd>(valueBuffer, mDimension);
}

//=============================================================================
Eigen::Map<const Eigen::VectorXd> Rn::getValue(
    const State *_state) const
{
  auto valueBuffer = reinterpret_cast<const double *>(
      reinterpret_cast<const unsigned char *>(_state));

  return Eigen::Map<const Eigen::VectorXd>(valueBuffer, mDimension);
}

//=============================================================================
void Rn::setValue(State *_state,
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
size_t Rn::getStateSizeInBytes() const
{
  return mDimension * sizeof(double);
}

//=============================================================================
StateSpace::State *Rn::allocateStateInBuffer(
    void *_buffer) const
{
  auto state = reinterpret_cast<State *>(_buffer);
  getMutableValue(state).setZero();
  return state;
}

//=============================================================================
void Rn::freeStateInBuffer(StateSpace::State */*_state*/) const
{
  // Do nothing.
}

//=============================================================================
void Rn::compose(const StateSpace::State *_state1,
                                   const StateSpace::State *_state2,
                                   StateSpace::State *_out) const
{
  // TODO: Disable this in release mode.
  if (_state1 == _out || _state2 == _out)
    throw std::invalid_argument("Output aliases input.");

  auto state1 = static_cast<const State *>(_state1);
  auto state2 = static_cast<const State *>(_state2);
  auto out = static_cast<State *>(_out);

  setValue(out, getValue(state1) + getValue(state2));
}

//=============================================================================
size_t Rn::getDimension() const
{
  return mDimension;
}

//=============================================================================
void Rn::getIdentity(StateSpace::State *_out) const
{
  auto out = static_cast<State *>(_out);
  setValue(out, Eigen::VectorXd::Zero(mDimension));
}

//=============================================================================
void Rn::getInverse(const StateSpace::State *_in,
                                      StateSpace::State *_out) const
{
  // TODO: Disable this in release mode.
  if (_out == _in)
    throw std::invalid_argument("Output aliases input.");

  auto in = static_cast<const State *>(_in);
  auto out = static_cast<State *>(_out);

  setValue(out, -getValue(in));
}

//=============================================================================
void Rn::copyState(
  const StateSpace::State *_source, StateSpace::State *_destination) const
{
  auto destination = static_cast<State *>(_destination);
  auto source = static_cast<const State *>(_source);
  setValue(destination, getValue(source));
}

//=============================================================================
void Rn::expMap(const Eigen::VectorXd &_tangent,
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

//=============================================================================
void Rn::logMap(const StateSpace::State *_in,
                                  Eigen::VectorXd &_tangent) const
{
  if (_tangent.size() != mDimension) {
    _tangent.resize(mDimension);
  }

  auto in = static_cast<const State *>(_in);
  _tangent = getValue(in);
}

//=============================================================================
void Rn::print(const StateSpace::State *_state, std::ostream &_os) const 
{
    auto val = getValue(static_cast<const State*>(_state));
    
    Eigen::IOFormat cleanFmt(3, Eigen::DontAlignCols, ",", ",", "", "", "[", "]");
    _os << val.format(cleanFmt);
}

}  // namespace statespace
}  // namespace aikido
