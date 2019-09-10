#include "aikido/statespace/SE3.hpp"

#include <dart/math/Geometry.hpp>

namespace aikido {
namespace statespace {
//==============================================================================
SE3::State::State() : mTransform(Isometry3d::Identity())
{
}

//==============================================================================
SE3::State::State(const Isometry3d& _transform) : mTransform(_transform)
{
}

//==============================================================================
auto SE3::State::getIsometry() const -> const Isometry3d&
{
  return mTransform;
}

//==============================================================================
void SE3::State::setIsometry(const Isometry3d& _transform)
{
  mTransform = _transform;
}

//==============================================================================
auto SE3::createState() const -> ScopedState
{
  return ScopedState(this);
}

//==============================================================================
SE3::ScopedState SE3::cloneState(const StateSpace::State* stateIn) const
{
  auto newState = createState();
  copyState(stateIn, newState);

  return newState;
}

//==============================================================================
auto SE3::getIsometry(const State* _state) const -> const Isometry3d&
{
  return _state->getIsometry();
}

//==============================================================================
void SE3::setIsometry(State* _state, const Isometry3d& _transform) const
{
  _state->setIsometry(_transform);
}

//==============================================================================
std::size_t SE3::getStateSizeInBytes() const
{
  return sizeof(State);
}

//==============================================================================
StateSpace::State* SE3::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//==============================================================================
void SE3::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//==============================================================================
void SE3::compose(
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

  out->mTransform = state1->mTransform * state2->mTransform;
}

//==============================================================================
void SE3::getIdentity(StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);
  setIsometry(out, Isometry3d::Identity());
}

//==============================================================================
void SE3::getInverse(
    const StateSpace::State* _in, StateSpace::State* _out) const
{
  // TODO: Disable this in release mode.
  if (_out == _in)
    throw std::invalid_argument("Output aliases input.");

  auto in = static_cast<const State*>(_in);
  auto out = static_cast<State*>(_out);
  setIsometry(out, getIsometry(in).inverse());
}

//==============================================================================
std::size_t SE3::getDimension() const
{
  return 6;
}

//==============================================================================
void SE3::copyState(
    const StateSpace::State* _source, StateSpace::State* _destination) const
{
  auto source = static_cast<const State*>(_source);
  auto dest = static_cast<State*>(_destination);
  setIsometry(dest, getIsometry(source));
}

//==============================================================================
void SE3::expMap(const Eigen::VectorXd& _tangent, StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);

  // TODO: Skip these checks in release mode.
  if (_tangent.rows() != 6)
  {
    std::stringstream msg;
    msg << "_tangent has incorrect size: expected 6"
        << ", got " << _tangent.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  Eigen::Isometry3d transform = dart::math::expMap(_tangent);
  out->mTransform = transform;
}

//==============================================================================
void SE3::logMap(const StateSpace::State* _in, Eigen::VectorXd& _tangent) const
{
  // TODO: Skip these checks in release mode.
  if (_tangent.rows() != 6)
  {
    _tangent.resize(6);
  }

  auto in = static_cast<const State*>(_in);
  Eigen::Isometry3d transform = getIsometry(in);

  _tangent = dart::math::logMap(transform);
}

//==============================================================================
void SE3::print(const StateSpace::State* _state, std::ostream& _os) const
{
  auto state = static_cast<const State*>(_state);
  auto transform = getIsometry(state);
  auto t = transform.translation();

  Eigen::VectorXd vals(7);
  Eigen::Quaterniond quat(transform.rotation());
  vals << quat.w(), quat.x(), quat.y(), quat.z(), t(0), t(1), t(2);

  Eigen::IOFormat cleanFmt(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "[", "]");
  _os << vals.format(cleanFmt);
}

} // namespace statespace
} // namespace aikido
