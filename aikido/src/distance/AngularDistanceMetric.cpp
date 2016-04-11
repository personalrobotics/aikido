#include <aikido/distance/AngularDistanceMetric.hpp>
#include <boost/math/constants/constants.hpp>

namespace aikido
{
namespace distance
{
AngularDistanceMetric::AngularDistanceMetric(
    std::shared_ptr<statespace::SO2StateSpace> _space)
    : mStateSpace(std::move(_space))
{
}

double AngularDistanceMetric::distance(
    const aikido::statespace::StateSpace::State* _state1,
    const aikido::statespace::StateSpace::State* _state2) const
{
  // Difference between angles
  double diff =
      mStateSpace->getAngle(
          static_cast<const statespace::SO2StateSpace::State*>(_state1))
      - mStateSpace->getAngle(
            static_cast<const statespace::SO2StateSpace::State*>(_state2));
  diff = fmod(fabs(diff), 2.0 * boost::math::constants::pi<double>());
  if (diff > boost::math::constants::pi<double>())
    diff -= 2.0 * boost::math::constants::pi<double>();
  return fabs(diff);
}

void AngularDistanceMetric::interpolate(
    const aikido::statespace::StateSpace::State* _from,
    const aikido::statespace::StateSpace::State* _to, const double _t,
    aikido::statespace::StateSpace::State* _state) const
{
  auto from = static_cast<const statespace::SO2StateSpace::State*>(_from);
  auto to = static_cast<const statespace::SO2StateSpace::State*>(_to);
  auto state = static_cast<statespace::SO2StateSpace::State*>(_state);
  double a =
      (1 - _t) * mStateSpace->getAngle(from) + _t * mStateSpace->getAngle(to);

  // TODO: Wrap?
  mStateSpace->setAngle(state, a);
}
}
}
