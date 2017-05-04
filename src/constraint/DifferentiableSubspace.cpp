#include <sstream>
#include <aikido/constraint/DifferentiableSubspace.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
DifferentiableSubspace::DifferentiableSubspace(
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

  if (_index >= mStateSpace->getNumSubspaces())
  {
    std::stringstream msg;
    msg << "Subspace Index " << _index << " is out of range [0, "
        << mStateSpace->getNumSubspaces() << "].";
    throw std::invalid_argument(msg.str());
  }

  if (mConstraint->getStateSpace() != mStateSpace->getSubspace<>(mIndex))
    throw std::invalid_argument(
      "Constraint does not apply to the specified Subspace.");
}

//=============================================================================
statespace::StateSpacePtr DifferentiableSubspace::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::vector<ConstraintType> DifferentiableSubspace::getConstraintTypes() const
{
  return mConstraint->getConstraintTypes();
}

//=============================================================================
size_t DifferentiableSubspace::getConstraintDimension() const
{
  return mConstraint->getConstraintDimension();
}

//=============================================================================
void DifferentiableSubspace::getValue(
  const statespace::StateSpace::State* _s,
  Eigen::VectorXd& _out) const
{
  auto state = static_cast<const statespace::CartesianProduct::State*>(_s);
  auto substate = mStateSpace->getSubState<>(state, mIndex);
  mConstraint->getValue(substate, _out);
}

//=============================================================================
void DifferentiableSubspace::getJacobian(
  const statespace::StateSpace::State* _s,
  Eigen::MatrixXd& _out) const
{
  auto state = static_cast<const statespace::CartesianProduct::State*>(_s);
  auto substate = mStateSpace->getSubState<>(state, mIndex);
  return mConstraint->getJacobian(substate, _out);
}

//=============================================================================
void  DifferentiableSubspace::getValueAndJacobian(
  const statespace::StateSpace::State* _s,
  Eigen::VectorXd& _val, Eigen::MatrixXd& _jac) const
{
  auto state = static_cast<const statespace::CartesianProduct::State*>(_s);
  auto substate = mStateSpace->getSubState<>(state, mIndex);
  return mConstraint->getValueAndJacobian(substate, _val, _jac);
}

} // namespace constraint
} // namespace aikido
