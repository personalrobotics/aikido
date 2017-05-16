#include <boost/math/constants/constants.hpp>
#include <aikido/distance/SE2.hpp>

namespace aikido {
namespace distance {

//=============================================================================
SE2::SE2(std::shared_ptr<statespace::SE2> _space)
  : mStateSpace(std::move(_space))
{
  if (mStateSpace == nullptr)
    throw std::invalid_argument("_space is nullptr.");
}

//=============================================================================
statespace::StateSpacePtr SE2::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
double SE2::distance(
    const aikido::statespace::StateSpace::State* _state1,
    const aikido::statespace::StateSpace::State* _state2) const
{
  Eigen::VectorXd tangent1;
  mStateSpace->logMap(
      static_cast<const statespace::SE2::State*>(_state1), tangent1);

  Eigen::VectorXd tangent2;
  mStateSpace->logMap(
      static_cast<const statespace::SE2::State*>(_state2), tangent2);

  Eigen::Vector3d diff;

  // Difference between angles
  double angleDiff = tangent1(0) - tangent2(0);
  angleDiff = std::fmod(std::abs(angleDiff), 2.0 * M_PI);
  if (angleDiff > M_PI)
    angleDiff -= 2.0 * M_PI;
  diff(0) = angleDiff;

  // Difference between R^2 positions
  diff.tail<2>() = tangent1.tail<2>() - tangent2.tail<2>();

  return diff.norm();
}

} // namespace distance
} // namespace aikido
