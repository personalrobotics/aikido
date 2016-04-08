#include <aikido/statespace/SE2StateSpace.hpp>
#include <Eigen/Geometry>

namespace aikido {
namespace statespace {

//=============================================================================
SE2StateSpace::State::State()
  : mTransform(Isometry2d::Identity())
{
}

//=============================================================================
SE2StateSpace::State::State(const Isometry2d& _transform)
  : mTransform(_transform)
{
}

//=============================================================================
auto SE2StateSpace::State::getIsometry() const 
  -> const Isometry2d&
{
  return mTransform;
}

//=============================================================================
void SE2StateSpace::State::setIsometry(
  const Isometry2d& _transform)
{
  mTransform = _transform;
}

//=============================================================================
auto SE2StateSpace::createState() const -> ScopedState
{
  return ScopedState(this);
}

//=============================================================================
auto SE2StateSpace::getIsometry(const State* _state) const
  -> const Isometry2d&
{
  return _state->getIsometry();
}

//=============================================================================
void SE2StateSpace::setIsometry(
  State* _state, const Isometry2d& _transform) const
{
  _state->setIsometry(_transform);
}


//=============================================================================
size_t SE2StateSpace::getStateSizeInBytes() const
{
  return sizeof(State);
}

//=============================================================================
StateSpace::State* SE2StateSpace::allocateStateInBuffer(void* _buffer) const
{
  return new (_buffer) State;
}

//=============================================================================
void SE2StateSpace::freeStateInBuffer(StateSpace::State* _state) const
{
  static_cast<State*>(_state)->~State();
}

//=============================================================================
auto SE2StateSpace::createSampleableConstraint(
  std::unique_ptr<util::RNG> _rng) const -> SampleableConstraintPtr
{
  throw std::runtime_error(
    "SE2StateSpace::createSampleableConstraint is not implemented.");
}

//=============================================================================
void SE2StateSpace::compose(
  const StateSpace::State* _state1, const StateSpace::State* _state2,
  StateSpace::State* _out) const
{
  auto state1 = static_cast<const State*>(_state1);
  auto state2 = static_cast<const State*>(_state2);
  auto out = static_cast<State*>(_out);

  out->mTransform = state1->mTransform * state2->mTransform;
}

} // namespace statespace
} // namespace aikido
