#include <aikido/statespace/GeodesicInterpolator.hpp>

namespace aikido {
namespace statespace {

//==============================================================================
GeodesicInterpolator::GeodesicInterpolator(
    statespace::ConstStateSpacePtr _stateSpace)
  : mStateSpace(std::move(_stateSpace))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is null.");
}

//==============================================================================
statespace::ConstStateSpacePtr GeodesicInterpolator::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::size_t GeodesicInterpolator::getNumDerivatives() const
{
  return 1;
}

//==============================================================================
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

//==============================================================================
void GeodesicInterpolator::interpolate(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to,
    double _alpha,
    statespace::StateSpace::State* _out) const
{
  const auto tangentVector = getTangentVector(_from, _to);

  auto relativeState = mStateSpace->createState();
  mStateSpace->expMap(_alpha * tangentVector, relativeState);

  mStateSpace->compose(_from, relativeState, _out);
}

//==============================================================================
void GeodesicInterpolator::getDerivative(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to,
    std::size_t _derivative,
    double /*_alpha*/,
    Eigen::VectorXd& _tangentVector) const
{
  if (_derivative == 0)
    throw std::invalid_argument("Derivative must be greater than zero.");
  else if (_derivative == 1)
    _tangentVector = getTangentVector(_from, _to);
  else
  {
    _tangentVector.resize(mStateSpace->getDimension());
    _tangentVector.setZero();
  }
}

} // namespace statespace
} // namespace aikido
