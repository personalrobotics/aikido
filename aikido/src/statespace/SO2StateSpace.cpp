#include <aikido/statespace/SO2StateSpace.hpp>

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
void SO2StateSpace::State::setRotation(const Eigen::Rotation2Dd& _rotation)
{
  mAngle = _rotation.angle();
}

//=============================================================================
int SO2StateSpace::getRepresentationDimension() const
{
  return 1;
}

//=============================================================================
void SO2StateSpace::compose(
  const StateSpace::State& _state1, const StateSpace::State& _state2,
	StateSpace::State& _out) const
{
  const auto& state1 = static_cast<const State&>(_state1);
  const auto& state2 = static_cast<const State&>(_state2);
  auto& out = static_cast<State&>(_out);

  out.mAngle = state1.mAngle + state2.mAngle;
}

} // namespace statespace
} // namespace aikido
