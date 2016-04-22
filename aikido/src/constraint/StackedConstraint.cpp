#include <aikido/constraint/StackedConstraint.hpp>
namespace aikido {
namespace constraint {


//=============================================================================
StackedConstraint::StackedConstraint(
  std::vector<DifferentiablePtr> _constraints,
  std::shared_ptr<aikido::statespace::StateSpace> _stateSpace)
: mConstraints(std::move(_constraints))
, mStateSpace(std::move(_stateSpace))
{
  if (mConstraints.size() == 0)
    throw std::invalid_argument("_constraints should not be empty.");
  
  if (!mStateSpace)
    throw std::invalid_argument("_stateSpace is nullptr.");

  // TODO: This checks pointer equality.
  for (auto constraint: mConstraints)
  {
    if (!constraint)
      throw std::invalid_argument("_constraints constaints nullptr.");

    if (constraint->getStateSpace().get() != mStateSpace.get())
    {
      std::stringstream msg;
      msg << "All constraints should have same statespace as mStatespace.";
      throw std::invalid_argument(msg.str());
    }
  }
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
    value.segment(index, dim) = constraint->getValue(_s);
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
    jacobian.middleRows(index, jac.rows()) = jac;
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

    value.segment(index, constraintDim) = pair.first;
    jacobian.middleRows(index, constraintDim) = pair.second;

    index += pair.second.rows();
  }

  return std::make_pair(value, jacobian);
}


//=============================================================================
std::vector<ConstraintType> StackedConstraint::getConstraintTypes() const
{
  std::vector<ConstraintType> constraintTypes;
  constraintTypes.reserve(mConstraints.size());

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
  return mStateSpace;
}

}
}
