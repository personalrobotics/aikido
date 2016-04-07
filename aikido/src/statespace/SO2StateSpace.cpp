#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/SO2StateSpaceSampleableConstraint.hpp>

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
constraint::SampleableConstraintPtr SO2StateSpace::createSampleableConstraint(
  std::unique_ptr<util::RNG> _rng) const
{
  return std::make_shared<SO2StateSpaceSampleableConstraint>(
    nullptr, // TODO: this should be enable_shared_from_this
    std::move(_rng));
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

} // namespace statespace
} // namespace aikido
