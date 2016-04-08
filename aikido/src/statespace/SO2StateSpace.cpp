#include <aikido/statespace/SO2StateSpace.hpp>
#include <boost/math/constants/constants.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
SO2StateSpace::State::State()
  : mAngle(0.)
{
}

//=============================================================================
SO2StateSpace::State::State(double _angle)
  : mAngle(_angle)
{
}

//=============================================================================
double SO2StateSpace::State::getAngle() const
{
  return mAngle;
}

//=============================================================================
void SO2StateSpace::State::setAngle(double _angle)
{
  mAngle = _angle;
}

//=============================================================================
Eigen::Rotation2Dd SO2StateSpace::State::getRotation() const
{
  return Eigen::Rotation2Dd(mAngle);
}

//=============================================================================
void SO2StateSpace::State::setRotation(
  const Eigen::Rotation2Dd& _rotation)
{
  mAngle = _rotation.angle();
}

//=============================================================================
auto SO2StateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
double SO2StateSpace::getAngle(const State* _state) const
{
  return _state->getAngle();
}

//=============================================================================
void SO2StateSpace::setAngle(State* _state, double _angle) const
{
  _state->setAngle(_angle);
}

//=============================================================================
Eigen::Rotation2Dd SO2StateSpace::getRotation(const State* _state) const
{
  return Eigen::Rotation2Dd(_state->mAngle);
}

//=============================================================================
void SO2StateSpace::setRotation(
  State* _state, const Eigen::Rotation2Dd& _rotation) const
{
  _state->mAngle = _rotation.angle();
}

//=============================================================================
size_t SO2StateSpace::getStateSizeInBytes() const
{
  return sizeof(State);
}

//=============================================================================
StateSpace::State* SO2StateSpace::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//=============================================================================
void SO2StateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//=============================================================================
void SO2StateSpace::compose(
  const StateSpace::State* _state1, const StateSpace::State* _state2,
	StateSpace::State* _out) const
{
  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  out->mAngle = state1->mAngle + state2->mAngle;
}

//=============================================================================
unsigned int SO2StateSpace::getDimension() const 
{
    return 1;
}

//=============================================================================
double SO2StateSpace::getMaximumExtent() const 
{
    return boost::math::constants::pi<double>();
}

//=============================================================================
double SO2StateSpace::getMeasure() const 
{
    return 2.0*boost::math::constants::pi<double>(); // OMPL
}

//=============================================================================
void SO2StateSpace::enforceBounds(StateSpace::State* _state) const 
{
    return;
}

//=============================================================================
bool SO2StateSpace::satisfiesBounds(const StateSpace::State* _state) const 
{
    return true;
}

//=============================================================================
void SO2StateSpace::copyState(StateSpace::State* _destination,
                              const StateSpace::State* _source) const
{
    auto source = static_cast<const State*>(_source);
    auto destination = static_cast<State*>(_destination);
    setAngle(destination, getAngle(source));
}

//=============================================================================
double SO2StateSpace::distance(const StateSpace::State* _state1,
                               const StateSpace::State* _state2) const
{
    // Difference between angles
    double diff = getAngle(static_cast<const State*>(_state1)) - 
        getAngle(static_cast<const State*>(_state2));
    diff = fmod(fabs(diff), 2.0*boost::math::constants::pi<double>());
    if(diff > boost::math::constants::pi<double>())
        diff -= 2.0*boost::math::constants::pi<double>();
    return diff;
}

//=============================================================================
bool SO2StateSpace::equalStates(const StateSpace::State* _state1,
                                const StateSpace::State* _state2) const
{
    double dist = distance(_state1, _state2);
    return dist < std::numeric_limits<double>::epsilon();
}

//=============================================================================
void SO2StateSpace::interpolate(const StateSpace::State* _from,
                                const StateSpace::State* _to,
                                const double _t,
                                StateSpace::State* _state) const
{
    auto st = static_cast<State*>(_state);
    double dist = distance(_from, _to);
    double a = getAngle(st) + _t*dist;

    // TODO: Wrap?
    setAngle(st, a);
}

} // namespace statespace
} // namespace aikido
