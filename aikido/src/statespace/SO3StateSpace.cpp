#include <aikido/statespace/SO3StateSpace.hpp>
#include <boost/math/constants/constants.hpp>
#include <dart/math/Geometry.h>
#include <iostream>

namespace aikido {
namespace statespace {

//=============================================================================
SO3StateSpace::SO3StateSpace::State::State()
  : mValue(1., 0., 0., 0.)
{
}

//=============================================================================
SO3StateSpace::SO3StateSpace::State::State(const Quaternion& _quaternion)
  : mValue(_quaternion)
{
  // TODO: Check if normalized.
}

//=============================================================================
auto SO3StateSpace::State::getQuaternion() const -> const Quaternion&
{
  return mValue;
}

//=============================================================================
void SO3StateSpace::State::setQuaternion(const Quaternion& _quaternion)
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
auto SO3StateSpace::getQuaternion(const State* _state) const
  -> const Quaternion&
{
  return _state->mValue;
}

//=============================================================================
void SO3StateSpace::setQuaternion(
  State* _state, const Quaternion& _quaternion) const
{
  _state->mValue = _quaternion;
}

//=============================================================================
size_t SO3StateSpace::getStateSizeInBytes() const
{
  return sizeof(State);
}

//=============================================================================
StateSpace::State* SO3StateSpace::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//=============================================================================
void SO3StateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//=============================================================================
void SO3StateSpace::compose(
  const StateSpace::State* _state1, const StateSpace::State* _state2,
  StateSpace::State* _out) const
{
  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  out->mValue = state1->mValue * state2->mValue;
}

//=============================================================================
unsigned int SO3StateSpace::getDimension() const 
{
    return 3;
}

//=============================================================================
double SO3StateSpace::getMaximumExtent() const 
{
    return 0.5 * boost::math::constants::pi<double>(); // OMPL
}

//=============================================================================
double SO3StateSpace::getMeasure() const 
{
    // half of the surface area of a unit 3-sphere
    return boost::math::constants::pi<double>() * boost::math::constants::pi<double>(); //OMPL
}

//=============================================================================
bool SO3StateSpace::satisfiesBounds(const StateSpace::State* _state) const 
{
    return true;
}

//=============================================================================
void SO3StateSpace::copyState(StateSpace::State* _destination,
                              const StateSpace::State* _source) const
{
    auto destination = static_cast<State*>(_destination);
    auto source = static_cast<const State*>(_source);
    
    setQuaternion(destination, getQuaternion(source));
}

//=============================================================================
double SO3StateSpace::distance(const StateSpace::State* _state1,
                               const StateSpace::State* _state2) const
{
    auto state1 = static_cast<const State*>(_state1);
    auto state2 = static_cast<const State*>(_state2);
    double r = getQuaternion(state1).dot(getQuaternion(state2));
    if(r < -1.0 || r > 1.0)
        return 0.0;
    r = acos(r);
    if(r <= 2.*boost::math::constants::pi<double>())
    {
        return r;
    }
    else
    {
        return boost::math::constants::pi<double>() - r;
    }
}

//=============================================================================
bool SO3StateSpace::equalStates(const StateSpace::State* _state1,
                                const StateSpace::State* _state2) const
{
    return distance(_state1, _state2) < std::numeric_limits<double>::epsilon();
}

//=============================================================================
void SO3StateSpace::interpolate(const StateSpace::State* _from,
                                const StateSpace::State* _to,
                                const double _t,
                                StateSpace::State* _state) const
{

    double dist = distance(_from, _to);
    if(dist > std::numeric_limits<double>::epsilon())
    {
        double d = 1.0 / dist;
        double s0 = sin( (1.0 - _t) * dist);
        double s1 = sin(_t * dist);

        auto from = static_cast<const State*>(_from);
        auto to = static_cast<const State*>(_to);
        
        Quaternion f = getQuaternion(from);
        Quaternion t = getQuaternion(to);
        double dq = f.dot(t);
        if(dq < 0.0)
            s1 *= -1.0;
        Quaternion iq;
        iq.x() = d * ( f.x() * s0 + t.x() * s1 );
        iq.y() = d * ( f.y() * s0 + t.y() * s1 );
        iq.z() = d * ( f.z() * s0 + t.z() * s1 );
        iq.w() = d * ( f.w() * s0 + t.w() * s1 );

        auto state = static_cast<State*>(_state);
        setQuaternion(state, iq);
    }
    else
    {
        copyState(_state, _from);
    }
}

} // namespace statespace
} // namespace aikido
