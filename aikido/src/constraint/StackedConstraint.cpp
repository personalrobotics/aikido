#include <aikido/constraint/StackedConstraint.hpp>
namespace aikido {
namespace constraint {


//=============================================================================
StackedConstraint::StackedConstraint(
  const std::vector<DifferentiablePtr>& _constraints,
  const std::shared_ptr<aikido::statespace::StateSpace> _stateSpace)
: mConstraints(_constraints)
, mStateSpace(_stateSpace)
{
  if (mConstraints.size() == 0)
  {
    throw std::invalid_argument("_constraint should not be empty.");
  }

  // TODO: Check if all constraints have the same statespace type.
}


//=============================================================================
size_t StackedConstraint::getConstraintDimension() const
{
  int dim = 0;

  for (auto constraint: mConstraints)
  {
    dim += constraint->getConstraintDimension();
  }

  return dim;
}


//=============================================================================
Eigen::VectorXd StackedConstraint::getValue(
  const statespace::StateSpace::State* _s) const
{
  int dimension = getConstraintDimension();
  
  Eigen::VectorXd value(dimension);

  int index = 0;
  for (auto constraint: mConstraints)
  {
    int dim = constraint->getConstraintDimension();
    value.block(index, 0, dim, 1) = constraint->getValue(_s);
    index += dim;
  }

  return value;
}


//=============================================================================
Eigen::MatrixXd StackedConstraint::getJacobian(
  const statespace::StateSpace::State* _s) const
{
  int constraintsDim = getConstraintDimension();
  int statesDim = mStateSpace->getDimension();

  Eigen::MatrixXd jacobian(constraintsDim, statesDim);

  int index = 0;
  for (auto constraint: mConstraints)
  {
    Eigen::MatrixXd jac = constraint->getJacobian(_s);
    jacobian.block(index, 0, jac.rows(), statesDim) = jac;
    index += jac.rows();
  }

  return jacobian;
}

//=============================================================================
std::pair<Eigen::VectorXd, Eigen::MatrixXd> StackedConstraint::getValueAndJacobian(
  const statespace::StateSpace::State* _s) const
{
  int constraintsDim = getConstraintDimension();
  int statesDim = mStateSpace->getDimension();

  Eigen::VectorXd value(constraintsDim);
  Eigen::MatrixXd jacobian(constraintsDim, statesDim);

  int index = 0;
  for (auto constraint: mConstraints)
  {
    int constraintDim = constraint->getConstraintDimension();

    // Get (Eigen::VectorXd value, Eigen::MatrixXd jacobian) pair.
    auto pair = constraint->getValueAndJacobian(_s);

    value.block(index, 0, constraintDim, 1) = pair.first;
    jacobian.block(index, 0, constraintDim, statesDim) = pair.second;

    index += pair.second.rows();
  }

  return std::make_pair(value, jacobian);
}


//=============================================================================
std::vector<ConstraintType> StackedConstraint::getConstraintTypes() const
{
  std::vector<ConstraintType> constraintTypes;

  for (auto constraint: mConstraints)
  {
    std::vector<ConstraintType> cTypes = constraint->getConstraintTypes();
    constraintTypes.insert(
      std::end(constraintTypes),
      std::begin(cTypes), std::end(cTypes));
  }

  return constraintTypes;
}


//=============================================================================
statespace::StateSpacePtr StackedConstraint::getStateSpace() const
{
  return mConstraints[0]->getStateSpace();
}

}
}
