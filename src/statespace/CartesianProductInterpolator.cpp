#include <aikido/statespace/CartesianProductInterpolator.hpp>

namespace aikido {
namespace statespace {

//==============================================================================
CartesianProductInterpolator::CartesianProductInterpolator(
    statespace::CartesianProductPtr _stateSpace,
    std::vector<InterpolatorPtr> interpolators)
  : mStateSpace(std::move(_stateSpace))
  , mInterpolators(interpolators)
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is null.");
  
  if(!static_cast<CartesianProductPtr>(mStateSpace))
    throw std::invalid_argument("StateSpace is not cartesian");
  
  if(mInterpolators.size() != mStateSpace->getNumSubspaces())
  {
    throw std::invalid_argument("Number of interpolators does not match number"
     "of statespaces");
  }

  for(std::size_t i = 0; i < mStateSpace->getNumSubspaces(); i++)
  {
    if(mStateSpace->getSubspace(i) != mInterpolators[i]->getStateSpace())
    {
      throw std::invalid_argument("StateSpace for interpolators is not a part of"
       "Statespace");
    }
  }
}

//==============================================================================
statespace::ConstStateSpacePtr CartesianProductInterpolator::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::size_t CartesianProductInterpolator::getNumDerivatives() const
{
  return 1;
}

//==============================================================================
Eigen::VectorXd CartesianProductInterpolator::getTangentVector(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to) const
{
  auto stateFrom = static_cast<const CartesianProduct::State*>(_from);
  if(stateFrom == nullptr)
  {
    throw std::runtime_error("State is not Cartesian Product of different states");
  }

  auto stateTo = static_cast<const CartesianProduct::State*>(_to);
  if(stateTo == nullptr)
  {
    throw std::runtime_error("State is not Cartesian Product of different states");
  }

  Eigen::VectorXd tangentVector;

  for(std::size_t i = 0; i < mStateSpace->getNumSubspaces(); i++)
  {
    auto subSpace = mInterpolators[i]->getStateSpace();

    auto fromInverse = subSpace->createState();
    subSpace->getInverse(mStateSpace->getSubState<>(stateFrom,i), fromInverse);

    auto toMinusFrom = subSpace->createState();
    subSpace->compose(fromInverse, mStateSpace->getSubState<>(stateTo,i), toMinusFrom);

    Eigen::VectorXd tangentSubVector;
    subSpace->logMap(toMinusFrom, tangentSubVector);

    tangentVector << tangentVector , tangentSubVector;
  }


  return tangentVector;
}

//==============================================================================
void CartesianProductInterpolator::interpolate(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to,
    double _alpha,
    statespace::StateSpace::State* _out) const
{
  auto stateFrom = static_cast<const CartesianProduct::State*>(_from);
  if(stateFrom == nullptr)
  {
    throw std::runtime_error("State is not Cartesian Product of different states");
  }

  auto stateTo = static_cast<const CartesianProduct::State*>(_to);
  if(stateTo == nullptr)
  {
    throw std::runtime_error("State is not Cartesian Product of different states");
  }

  auto stateOut = static_cast<CartesianProduct::State*>(_out);
  if(stateOut == nullptr)
  {
    throw std::runtime_error("State is not Cartesian Product of different states");
  }

  for(std::size_t i = 0; i < mStateSpace->getNumSubspaces(); i++)
  {
    mInterpolators[i]->interpolate(mStateSpace->getSubState<>(stateFrom,i),
                                  mStateSpace->getSubState<>(stateTo,i),
                                  _alpha,
                                  mStateSpace->getSubState<>(stateOut,i));
  }
}

//==============================================================================
void CartesianProductInterpolator::getDerivative(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to,
    std::size_t _derivative,
    double /*_alpha*/,
    Eigen::VectorXd& _tangentVector) const
{
  // TODO : Implement derivative
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
