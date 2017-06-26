#include <boost/math/constants/constants.hpp>
#include <aikido/distance/SE2.hpp>

namespace aikido {
namespace distance {

//=============================================================================
SE2::SE2(std::shared_ptr<statespace::SE2> _space,
         std::vector<double> _weights)
        : mStateSpace(std::move(_space)), mWeights(std::move(_weights))
{
  if (mStateSpace == nullptr)
  {
    throw std::invalid_argument("_space is nullptr.");
  }
  if (mWeights.size() != 2)
  {
    std::stringstream msg;
    msg << "Must provide a weight for each subspace in the "
           "Cartesian Semidirect Product. "
        << " (subspaces = 2 [R^2, SO(2)]"
        << " , weights = " << mWeights.size() << ")";
    throw std::invalid_argument(msg.str());
  }

  for (size_t i = 0; i < mWeights.size(); ++i)
  {
    if (mWeights[i] < 0)
    {
      std::stringstream msg;
      msg << "The weight at position " << i
          << " is negative. All weights must be positive.";
      throw std::invalid_argument(msg.str());
    }
  }
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

  Eigen::Vector2d linearDistance;
  double angularDistance;

  // Difference between angles
  angularDistance = tangent1(0) - tangent2(0);
  angularDistance = std::fmod(std::abs(angularDistance), 2.0 * M_PI);
  if (angularDistance > M_PI)
    angularDistance -= 2.0 * M_PI;

  // Difference between R^2 positions
  linearDistance = tangent1.tail<2>() - tangent2.tail<2>();

  return mWeights[0]*angularDistance + mWeights[1]*(linearDistance.norm());
}

} // namespace distance
} // namespace aikido
