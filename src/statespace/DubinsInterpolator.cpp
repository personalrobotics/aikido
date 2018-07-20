#include <aikido/statespace/DubinsInterpolator.hpp>
#include <aikido/statespace/SE2.hpp>
#include <aikido/statespace/DubinsPath.hpp>

namespace aikido {
namespace statespace {

//==============================================================================
DubinsInterpolator::DubinsInterpolator(
    statespace::ConstStateSpacePtr _stateSpace,
    double turningRadius)
  : mStateSpace(std::move(_stateSpace))
  , mTurningRadius(turningRadius)
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is null.");
  
  if(!dynamic_cast<const SE2*>(mStateSpace.get()))
    throw std::invalid_argument("StateSpace is not SE2");
}

//==============================================================================
statespace::ConstStateSpacePtr DubinsInterpolator::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::size_t DubinsInterpolator::getNumDerivatives() const
{
  return 1;
}

//==============================================================================
Eigen::VectorXd DubinsInterpolator::getTangentVector(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to) const
{
  //TODO: Implement
  auto fromInverse = mStateSpace->createState();
  mStateSpace->getInverse(_from, fromInverse);

  auto toMinusFrom = mStateSpace->createState();
  mStateSpace->compose(fromInverse, _to, toMinusFrom);

  Eigen::VectorXd tangentVector;
  mStateSpace->logMap(toMinusFrom, tangentVector);

  return tangentVector;
}

//==============================================================================
void DubinsInterpolator::interpolate(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to,
    double _alpha,
    statespace::StateSpace::State* _out) const
{
  auto state1 = static_cast<const statespace::SE2::State*>(_from);
  auto state2 = static_cast<const statespace::SE2::State*>(_to);

  common::DubinsPath path(*state1, *state2, mTurningRadius);

  auto outState = path.interpolate(_alpha);

  mStateSpace->copyState(static_cast<statespace::StateSpace::State*>(&outState) ,_out);

  return;
}

//==============================================================================
void DubinsInterpolator::getDerivative(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to,
    std::size_t _derivative,
    double /*_alpha*/,
    Eigen::VectorXd& _tangentVector) const
{
  //TODO: Implement
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
