#include <boost/math/constants/constants.hpp>
#include <aikido/distance/SE2Weighted.hpp>

namespace aikido {
namespace distance {

//==============================================================================
SE2Weighted::SE2Weighted(std::shared_ptr<const statespace::SE2> space)
  : mStateSpace(std::move(space)), mWeights(Eigen::Vector2d::Ones())
{
  if (mStateSpace == nullptr)
    throw std::invalid_argument("space is nullptr.");
}

//==============================================================================
SE2Weighted::SE2Weighted(
    std::shared_ptr<const statespace::SE2> space,
    const Eigen::Vector2d& weights)
  : mStateSpace(std::move(space)), mWeights(weights)
{
  if (mStateSpace == nullptr)
    throw std::invalid_argument("space is nullptr.");

  for (int i = 0; i < mWeights.size(); ++i)
  {
    if (mWeights[i] < 0.0)
    {
      std::stringstream msg;
      msg << "The weight at position " << i
          << " is negative. All weights must be positive.";
      throw std::invalid_argument(msg.str());
    }
  }
}

//==============================================================================
statespace::ConstStateSpacePtr SE2Weighted::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
double SE2Weighted::distance(
    const aikido::statespace::StateSpace::State* state1,
    const aikido::statespace::StateSpace::State* state2) const
{
  Eigen::VectorXd tangent1;
  mStateSpace->logMap(
      static_cast<const statespace::SE2::State*>(state1), tangent1);

  Eigen::VectorXd tangent2;
  mStateSpace->logMap(
      static_cast<const statespace::SE2::State*>(state2), tangent2);

  Eigen::Vector2d linearDistance;
  double angularDistance;

  // Difference between R^2 positions
  linearDistance = tangent2.head<2>() - tangent1.head<2>();

  // Difference between angles
  angularDistance = tangent2[2] - tangent1[2];
  angularDistance = std::fmod(std::abs(angularDistance), 2.0 * M_PI);
  if (angularDistance > M_PI)
    angularDistance = 2 * M_PI - angularDistance;

  return mWeights[0] * (linearDistance.norm()) + mWeights[1] * angularDistance;
}

} // namespace distance
} // namespace aikido
