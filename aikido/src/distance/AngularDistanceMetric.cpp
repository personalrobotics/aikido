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
  if (mStateSpace == nullptr) {
    throw std::invalid_argument("SO2StateSpace is nullptr.");
  }
}

statespace::StateSpacePtr AngularDistanceMetric::getStateSpace() const
{
  return mStateSpace;
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
  diff = std::fmod(std::fabs(diff), 2.0 * M_PI);
  if (diff > M_PI)
      diff -= 2.0 * M_PI;
  return std::fabs(diff);
}

}
}
