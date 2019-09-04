#include "aikido/constraint/DifferentiableIntersection.hpp"

namespace aikido {
namespace constraint {

//==============================================================================
DifferentiableIntersection::DifferentiableIntersection(
    std::vector<DifferentiablePtr> _constraints,
    std::shared_ptr<const aikido::statespace::StateSpace> _stateSpace)
  : mConstraints(std::move(_constraints)), mStateSpace(std::move(_stateSpace))
{
  if (mConstraints.size() == 0)
    throw std::invalid_argument("_constraints should not be empty.");

  if (!mStateSpace)
    throw std::invalid_argument("_stateSpace is nullptr.");

  // TODO: This checks pointer equality.
  for (auto constraint : mConstraints)
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

//==============================================================================
std::size_t DifferentiableIntersection::getConstraintDimension() const
{
  int dim = 0;

  for (auto constraint : mConstraints)
    dim += constraint->getConstraintDimension();

  return dim;
}

//==============================================================================
void DifferentiableIntersection::getValue(
    const statespace::StateSpace::State* _s, Eigen::VectorXd& _out) const
{
  int dimension = getConstraintDimension();

  _out.resize(dimension);

  int index = 0;
  for (auto constraint : mConstraints)
  {
    int dim = constraint->getConstraintDimension();
    Eigen::VectorXd out;
    constraint->getValue(_s, out);
    _out.segment(index, dim) = out;
    index += dim;
  }
}

//==============================================================================
void DifferentiableIntersection::getJacobian(
    const statespace::StateSpace::State* _s, Eigen::MatrixXd& _out) const
{
  int constraintsDim = getConstraintDimension();
  int statesDim = mStateSpace->getDimension();

  _out.resize(constraintsDim, statesDim);

  int index = 0;
  for (auto constraint : mConstraints)
  {
    Eigen::MatrixXd jac;
    constraint->getJacobian(_s, jac);
    _out.middleRows(index, jac.rows()) = jac;

    index += jac.rows();
  }
}

//==============================================================================
void DifferentiableIntersection::getValueAndJacobian(
    const statespace::StateSpace::State* _s,
    Eigen::VectorXd& _val,
    Eigen::MatrixXd& _jac) const
{
  int constraintsDim = getConstraintDimension();
  int statesDim = mStateSpace->getDimension();

  _val.resize(constraintsDim);
  _jac.resize(constraintsDim, statesDim);

  int index = 0;
  for (auto constraint : mConstraints)
  {
    int constraintDim = constraint->getConstraintDimension();

    // Get (Eigen::VectorXd value, Eigen::MatrixXd jacobian) pair.
    Eigen::VectorXd val;
    Eigen::MatrixXd jac;
    constraint->getValueAndJacobian(_s, val, jac);

    _val.segment(index, constraintDim) = val;
    _jac.middleRows(index, constraintDim) = jac;

    index += jac.rows();
  }
}

//==============================================================================
std::vector<ConstraintType> DifferentiableIntersection::getConstraintTypes()
    const
{
  std::vector<ConstraintType> constraintTypes;
  constraintTypes.reserve(mConstraints.size());

  for (auto constraint : mConstraints)
  {
    std::vector<ConstraintType> cTypes = constraint->getConstraintTypes();
    constraintTypes.insert(
        std::end(constraintTypes), std::begin(cTypes), std::end(cTypes));
  }

  return constraintTypes;
}

//==============================================================================
statespace::ConstStateSpacePtr DifferentiableIntersection::getStateSpace() const
{
  return mStateSpace;
}

} // namespace constraint
} // namespace aikido
