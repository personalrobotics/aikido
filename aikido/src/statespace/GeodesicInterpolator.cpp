#include <aikido/statespace/GeodesicInterpolator.hpp>

namespace aikido {
namespace statespace {

//=============================================================================
GeodesicInterpolator::GeodesicInterpolator(
      statespace::StateSpacePtr _stateSpace)
  : mStateSpace(std::move(_stateSpace))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is null.");
}

//=============================================================================
statespace::StateSpacePtr GeodesicInterpolator::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
Eigen::VectorXd GeodesicInterpolator::getTangentVector(
  const statespace::StateSpace::State* _from,
  const statespace::StateSpace::State* _to) const
{
  const auto fromInverse = mStateSpace->createState();
  mStateSpace->getInverse(_from, fromInverse);

  const auto toMinusFrom = mStateSpace->createState();
  mStateSpace->compose(fromInverse, _to, toMinusFrom);

  Eigen::VectorXd tangentVector;
  mStateSpace->logMap(toMinusFrom, tangentVector);

  return tangentVector;
}

//=============================================================================
void GeodesicInterpolator::interpolate(
  const statespace::StateSpace::State* _from,
  const statespace::StateSpace::State* _to, double _alpha,
  statespace::StateSpace::State* _out) const
{
  const auto tangentVector = getTangentVector(_from, _to);

  auto relativeState = mStateSpace->createState();
  mStateSpace->expMap(_alpha * tangentVector, relativeState);

  mStateSpace->compose(_from, relativeState, _out);
}

} // namespace statespace
} // namespace aikido
