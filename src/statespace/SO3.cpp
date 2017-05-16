#include <iostream>
#include <dart/math/Geometry.hpp>
#include <aikido/statespace/SO3.hpp>

namespace aikido {
namespace statespace {
//=============================================================================
SO3::SO3::State::State() : mValue(1., 0., 0., 0.)
{
}

//=============================================================================
SO3::SO3::State::State(const Quaternion& _quaternion) : mValue(_quaternion)
{
  // TODO: Check if normalized.
}

//=============================================================================
auto SO3::State::getQuaternion() const -> const Quaternion&
{
  return mValue;
}

//=============================================================================
void SO3::State::setQuaternion(const Quaternion& _quaternion)
{
  // TODO: Check if normalized.
  mValue = _quaternion;
}

//=============================================================================
auto SO3::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
auto SO3::getQuaternion(const State* _state) const -> const Quaternion&
{
  return _state->mValue;
}

//=============================================================================
void SO3::setQuaternion(State* _state, const Quaternion& _quaternion) const
{
  _state->mValue = _quaternion;
}

//=============================================================================
size_t SO3::getStateSizeInBytes() const
{
  return sizeof(State);
}

//=============================================================================
StateSpace::State* SO3::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//=============================================================================
void SO3::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//=============================================================================
void SO3::compose(
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

  out->mValue = state1->mValue * state2->mValue;
}

//=============================================================================
void SO3::getIdentity(StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);
  setQuaternion(out, Quaternion::Identity());
}

//=============================================================================
void SO3::getInverse(
    const StateSpace::State* _in, StateSpace::State* _out) const
{
  // TODO: Disable this in release mode.
  if (_out == _in)
    throw std::invalid_argument("Output aliases input.");

  auto in = static_cast<const State*>(_in);
  auto out = static_cast<State*>(_out);

  setQuaternion(out, getQuaternion(in).inverse());
}

//=============================================================================
size_t SO3::getDimension() const
{
  return 3;
}

//=============================================================================
void SO3::copyState(
    const StateSpace::State* _source, StateSpace::State* _destination) const
{
  auto destination = static_cast<State*>(_destination);
  auto source = static_cast<const State*>(_source);

  setQuaternion(destination, getQuaternion(source));
}

//=============================================================================
void SO3::expMap(const Eigen::VectorXd& _tangent, StateSpace::State* _out) const
{
  auto out = static_cast<State*>(_out);

  // TODO: Skip these checks in release mode.
  if (_tangent.rows() != 3)
  {
    std::stringstream msg;
    msg << "_tangent has incorrect size: expected 3"
        << ", got " << _tangent.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  Eigen::Vector6d tangent(Eigen::Vector6d::Zero());
  tangent.head<3>() = _tangent;

  Eigen::Isometry3d transform = dart::math::expMap(tangent);
  out->setQuaternion(Quaternion(transform.rotation()));
}

//=============================================================================
void SO3::logMap(const StateSpace::State* _in, Eigen::VectorXd& _tangent) const
{
  if (_tangent.rows() != 3)
  {
    _tangent.resize(3);
  }
  auto in = static_cast<const State*>(_in);

  // Compute rotation matrix from quaternion
  Eigen::Matrix3d rotMat = getQuaternion(in).toRotationMatrix();
  _tangent = dart::math::logMap(rotMat);
}

//=============================================================================
void SO3::print(const StateSpace::State* _state, std::ostream& _os) const
{
  auto state = static_cast<const State*>(_state);

  Eigen::IOFormat cleanFmt(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "[", "]");
  auto quat = getQuaternion(state);
  _os << Eigen::Vector4d(quat.w(), quat.x(), quat.y(), quat.z())
             .format(cleanFmt);
}

} // namespace statespace
} // namespace aikido
