#include <sstream>
#include <aikido/constraint/DifferentiableSubSpace.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
DifferentiableSubSpace::DifferentiableSubSpace(
      std::shared_ptr<statespace::CartesianProduct> _stateSpace,
      DifferentiablePtr _constraint, size_t _index)
  : mStateSpace(std::move(_stateSpace))
  , mConstraint(std::move(_constraint))
  , mIndex(_index)
{
  if (!mStateSpace)
    throw std::invalid_argument("CartesianProduct is nullptr.");

  if (!mConstraint)
    throw std::invalid_argument("Differentiable is nullptr.");

  if (_index >= mStateSpace->getNumStates())
  {
    std::stringstream msg;
    msg << "Subspace Index " << _index << " is out of range [0, "
        << mStateSpace->getNumStates() << "].";
    throw std::invalid_argument(msg.str());
  }

  if (mConstraint->getStateSpace() != mStateSpace->getSubSpace<>(mIndex))
    throw std::invalid_argument(
      "Constraint does not apply to the specified SubSpace.");
}

//=============================================================================
statespace::StateSpacePtr DifferentiableSubSpace::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::vector<ConstraintType> DifferentiableSubSpace::getConstraintTypes() const
{
  return mConstraint->getConstraintTypes();
}

//=============================================================================
size_t DifferentiableSubSpace::getConstraintDimension() const
{
  return mConstraint->getConstraintDimension();
}

//=============================================================================
Eigen::VectorXd DifferentiableSubSpace::getValue(
  const statespace::StateSpace::State* _s) const
{
  auto state = static_cast<const statespace::CartesianProduct::State*>(_s);
  auto substate = mStateSpace->getSubState<>(state, mIndex);
  return mConstraint->getValue(substate);
}

//=============================================================================
Eigen::MatrixXd DifferentiableSubSpace::getJacobian(
  const statespace::StateSpace::State* _s) const
{
  auto state = static_cast<const statespace::CartesianProduct::State*>(_s);
  auto substate = mStateSpace->getSubState<>(state, mIndex);
  return mConstraint->getJacobian(substate);
}

//=============================================================================
std::pair<Eigen::VectorXd, Eigen::MatrixXd>
  DifferentiableSubSpace::getValueAndJacobian(
    const statespace::StateSpace::State* _s) const
{
  auto state = static_cast<const statespace::CartesianProduct::State*>(_s);
  auto substate = mStateSpace->getSubState<>(state, mIndex);
  return mConstraint->getValueAndJacobian(substate);
}

} // constraint
} // aikido
