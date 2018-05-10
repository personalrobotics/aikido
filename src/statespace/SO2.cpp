#include <aikido/statespace/SO2.hpp>

namespace aikido {
namespace statespace {
//==============================================================================
SO2::State::State(double angle)
{
  setAngle(angle);
}

//==============================================================================
double SO2::State::getAngle() const
{
  return mAngle;
}

//==============================================================================
void SO2::State::setAngle(double angle)
{
  double boundedAngle = std::fmod(angle, 2.0 * M_PI);
  if (boundedAngle > M_PI)
    boundedAngle -= 2.0 * M_PI;
  if (boundedAngle < -M_PI)
    boundedAngle += 2.0 * M_PI;
  mAngle = boundedAngle;
}

//==============================================================================
Eigen::Rotation2Dd SO2::State::getRotation() const
{
  return Eigen::Rotation2Dd(mAngle);
}

//==============================================================================
void SO2::State::setRotation(const Eigen::Rotation2Dd& rotation)
{
  setAngle(rotation.angle());
}

//==============================================================================
auto SO2::createState() const -> ScopedState
{
  return ScopedState(this);
}

//==============================================================================
double SO2::getAngle(const State* state) const
{
  return state->getAngle();
}

//==============================================================================
void SO2::setAngle(State* state, double angle) const
{
  state->setAngle(angle);
}

//==============================================================================
Eigen::Rotation2Dd SO2::getRotation(const State* state) const
{
  return Eigen::Rotation2Dd(state->mAngle);
}

//==============================================================================
void SO2::setRotation(State* state, const Eigen::Rotation2Dd& rotation) const
{
  state->setAngle(rotation.angle());
}

//==============================================================================
std::size_t SO2::getStateSizeInBytes() const
{
  return sizeof(State);
}

//==============================================================================
StateSpace::State* SO2::allocateStateInBuffer(void* buffer) const
{
  return new (buffer) State;
}

//==============================================================================
void SO2::freeStateInBuffer(StateSpace::State* state) const
{
  static_cast<State*>(state)->~State();
}

//==============================================================================
void SO2::compose(
    const StateSpace::State* state1,
    const StateSpace::State* state2,
    StateSpace::State* out) const
{
#ifndef NDEBUG // Debug mode
  assert(state1 != out && state2 != out);
#endif

  auto sState1 = static_cast<const State*>(state1);
  auto sState2 = static_cast<const State*>(state2);
  auto sOut = static_cast<State*>(out);

  setAngle(sOut, sState1->mAngle + sState2->mAngle);
}

//==============================================================================
void SO2::getIdentity(StateSpace::State* out) const
{
  auto sOut = static_cast<State*>(out);
  setAngle(sOut, 0.);
}

//==============================================================================
void SO2::getInverse(const StateSpace::State* in, StateSpace::State* out) const
{
#ifndef NDEBUG // Debug mode
  assert(out != in);
#endif

  auto sIn = static_cast<const State*>(in);
  auto sOut = static_cast<State*>(out);

  setAngle(sOut, -getAngle(sIn));
}

//==============================================================================
std::size_t SO2::getDimension() const
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
void SO2::expMap(const Eigen::VectorXd& tangent, StateSpace::State* out) const
{
  auto sOut = static_cast<State*>(out);

#ifndef NDEBUG // Debug mode
  assert(tangent.rows() == 1);
#endif

  double angle = tangent(0);
  setAngle(sOut, angle);
}

//==============================================================================
void SO2::logMap(const StateSpace::State* in, Eigen::VectorXd& tangent) const
{
  if (tangent.rows() != 1)
    tangent.resize(1);

  auto sIn = static_cast<const State*>(in);
  tangent(0) = getAngle(sIn);
}

//==============================================================================
void SO2::print(const StateSpace::State* state, std::ostream& os) const
{
  auto sState = static_cast<const State*>(state);
  os << "[" << getAngle(sState) << "]";
}

} // namespace statespace
} // namespace aikido
