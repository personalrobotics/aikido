#include "aikido/statespace/SO2.hpp"

namespace aikido {
namespace statespace {

//==============================================================================
SO2::State::State(double angle)
{
  fromAngle(angle);
}

//==============================================================================
double SO2::State::toAngle() const
{
  return mAngle;
}

//==============================================================================
void SO2::State::fromAngle(double angle)
{
  double boundedAngle = std::fmod(angle, 2.0 * M_PI);
  if (boundedAngle > M_PI)
    boundedAngle -= 2.0 * M_PI;
  if (boundedAngle <= -M_PI)
    boundedAngle += 2.0 * M_PI;
  mAngle = boundedAngle;
}

//==============================================================================
Eigen::Rotation2Dd SO2::State::toRotation() const
{
  return Eigen::Rotation2Dd(mAngle);
}

//==============================================================================
void SO2::State::fromRotation(const Eigen::Rotation2Dd& rotation)
{
  fromAngle(rotation.angle());
}

//==============================================================================
auto SO2::createState() const -> ScopedState
{
  return ScopedState(this);
}

//==============================================================================
SO2::ScopedState SO2::cloneState(const StateSpace::State* stateIn) const
{
  auto newState = createState();
  copyState(stateIn, newState);

  return newState;
}

//==============================================================================
double SO2::toAngle(const State* state) const
{
  return state->toAngle();
}

//==============================================================================
void SO2::fromAngle(State* state, double angle) const
{
  state->fromAngle(angle);
}

//==============================================================================
Eigen::Rotation2Dd SO2::toRotation(const State* state) const
{
  return Eigen::Rotation2Dd(state->mAngle);
}

//==============================================================================
void SO2::fromRotation(State* state, const Eigen::Rotation2Dd& rotation) const
{
  state->fromAngle(rotation.angle());
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
void SO2::freeStateInBuffer(const StateSpace::State* state) const
{
  static_cast<const State*>(state)->~State();
}

//==============================================================================
void SO2::compose(
    const StateSpace::State* state1,
    const StateSpace::State* state2,
    StateSpace::State* out) const
{
  if (state1 == out || state2 == out)
    throw std::invalid_argument("Output aliases input.");

  auto sState1 = static_cast<const State*>(state1);
  auto sState2 = static_cast<const State*>(state2);
  auto sOut = static_cast<State*>(out);

  fromAngle(sOut, sState1->mAngle + sState2->mAngle);
}

//==============================================================================
void SO2::getIdentity(StateSpace::State* out) const
{
  auto sOut = static_cast<State*>(out);
  fromAngle(sOut, 0.);
}

//==============================================================================
void SO2::getInverse(const StateSpace::State* in, StateSpace::State* out) const
{
  if (out == in)
    throw std::invalid_argument("Output aliases input.");

  auto sIn = static_cast<const State*>(in);
  auto sOut = static_cast<State*>(out);

  fromAngle(sOut, -toAngle(sIn));
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
  fromAngle(destination, toAngle(source));
}

//==============================================================================
void SO2::expMap(const Eigen::VectorXd& tangent, StateSpace::State* out) const
{
  auto sOut = static_cast<State*>(out);

  if (tangent.rows() != 1)
  {
    std::stringstream msg;
    msg << "tangent has incorrect size: expected 1"
        << ", got " << tangent.rows() << ".\n";
    throw std::runtime_error(msg.str());
  }

  double angle = tangent(0);
  fromAngle(sOut, angle);
}

//==============================================================================
void SO2::logMap(const StateSpace::State* in, Eigen::VectorXd& tangent) const
{
  if (tangent.rows() != 1)
    tangent.resize(1);

  auto sIn = static_cast<const State*>(in);
  tangent(0) = toAngle(sIn);
}

//==============================================================================
void SO2::print(const StateSpace::State* state, std::ostream& os) const
{
  auto sState = static_cast<const State*>(state);
  os << "[" << toAngle(sState) << "]";
}

} // namespace statespace
} // namespace aikido
