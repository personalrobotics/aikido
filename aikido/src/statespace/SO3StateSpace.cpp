#include <aikido/statespace/SO3StateSpace.hpp>
#include <boost/math/constants/constants.hpp>
#include <dart/math/Geometry.h>
#include <iostream>

namespace aikido
{
namespace statespace
{
//=============================================================================
SO3StateSpace::SO3StateSpace::State::State()
    : mValue(1., 0., 0., 0.)
{
}

//=============================================================================
SO3StateSpace::SO3StateSpace::State::State(const Quaternion &_quaternion)
    : mValue(_quaternion)
{
  // TODO: Check if normalized.
}

//=============================================================================
auto SO3StateSpace::State::getQuaternion() const -> const Quaternion &
{
  return mValue;
}

//=============================================================================
void SO3StateSpace::State::setQuaternion(const Quaternion &_quaternion)
{
  // TODO: Check if normalized.
  mValue = _quaternion;
}

//=============================================================================
auto SO3StateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
auto SO3StateSpace::getQuaternion(const State *_state) const -> const Quaternion
    &
{
  return _state->mValue;
}

//=============================================================================
void SO3StateSpace::setQuaternion(State *_state,
                                  const Quaternion &_quaternion) const
{
  _state->mValue = _quaternion;
}

//=============================================================================
size_t SO3StateSpace::getStateSizeInBytes() const { return sizeof(State); }

//=============================================================================
StateSpace::State *SO3StateSpace::allocateStateInBuffer(void *_buffer) const
{
  return new (_buffer) State;
}

//=============================================================================
void SO3StateSpace::freeStateInBuffer(StateSpace::State *_state) const
{
  static_cast<State *>(_state)->~State();
}

//=============================================================================
void SO3StateSpace::compose(const StateSpace::State *_state1,
                            const StateSpace::State *_state2,
                            StateSpace::State *_out) const
{
  auto state1 = static_cast<const State *>(_state1);
  auto state2 = static_cast<const State *>(_state2);
  auto out = static_cast<State *>(_out);

  out->mValue = state1->mValue * state2->mValue;
}

//=============================================================================
void SO3StateSpace::getIdentity(StateSpace::State *_out) const
{
  auto out = static_cast<State *>(_out);
  setQuaternion(out, Quaternion::Identity());
}

//=============================================================================
void SO3StateSpace::getInverse(const StateSpace::State *_in,
                               StateSpace::State *_out) const
{
    auto in = static_cast<const State*>(_in);
    auto out = static_cast<State*>(_out);
    
    setQuaternion(out, getQuaternion(in).inverse());
}

//=============================================================================
unsigned int SO3StateSpace::getDimension() const { return 3; }

//=============================================================================
double SO3StateSpace::getMaximumExtent() const
{
  return 0.5 * boost::math::constants::pi<double>();  // OMPL
}

//=============================================================================
double SO3StateSpace::getMeasure() const
{
  // half of the surface area of a unit 3-sphere
  return boost::math::constants::pi<double>()
         * boost::math::constants::pi<double>();  // OMPL
}

//=============================================================================
void SO3StateSpace::copyState(StateSpace::State *_destination,
                              const StateSpace::State *_source) const
{
  auto destination = static_cast<State *>(_destination);
  auto source = static_cast<const State *>(_source);

  setQuaternion(destination, getQuaternion(source));
}

//=============================================================================
double SO3StateSpace::distance(const StateSpace::State *_state1,
                               const StateSpace::State *_state2) const
{
  auto state1 = static_cast<const State *>(_state1);
  auto state2 = static_cast<const State *>(_state2);
  return getQuaternion(state1).angularDistance(getQuaternion(state2));
}

//=============================================================================
bool SO3StateSpace::equalStates(const StateSpace::State *_state1,
                                const StateSpace::State *_state2) const
{
  // TODO: Make this a parameter.
  return distance(_state1, _state2) < 1e-7;
}

//=============================================================================
void SO3StateSpace::interpolate(const StateSpace::State *_from,
                                const StateSpace::State *_to, const double _t,
                                StateSpace::State *_state) const
{
  auto from = static_cast<const State *>(_from);
  auto to = static_cast<const State *>(_to);
  auto state = static_cast<State *>(_state);

  setQuaternion(state, getQuaternion(from).slerp(_t, getQuaternion(to)));
}

//=============================================================================
void SO3StateSpace::expMap(const Eigen::VectorXd &_tangent,
                           StateSpace::State *_out) const
{
  auto out = static_cast<State *>(_out);

  // TODO: Skip these checks in release mode.
  if (_tangent.rows() != 3) {
    std::stringstream msg;
    msg << "_tangent has incorrect size: expected 3"
        << ", got " << _tangent.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  Eigen::Vector6d tangent(Eigen::Vector6d::Zero());
  tangent.topRows(3) = _tangent;

  Eigen::Isometry3d transform = dart::math::expMap(tangent);
  out->setQuaternion(Quaternion(transform.rotation()));
}

}  // namespace statespace
}  // namespace aikido
