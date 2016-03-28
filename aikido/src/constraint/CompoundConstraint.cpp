#include <aikido/constraint/CompoundConstraint.hpp>

namespace aikido{
namespace constraint{

//=============================================================================
CompoundConstraint::CompoundConstraint(
  const std::vector<DifferentiablePtr>& _constraint)
: mConstraint(_constraint)
{
  if (mConstraint.size() == 0)
  {
    throw std::invalid_argument("_constraint is empty.");
  }
}

//=============================================================================
size_t CompoundConstraint::getConstraintDimension() const
{
  size_t dims = 0;
  for(int i = 0; i < mConstraint.size(); ++i)
  {
    dims += mConstraint.at(i)->getConstraintDimension();
  }

  return dim;
}

//=============================================================================
Eigen::VectorXd CompoundConstraint::getValue(const state::StatePtr& _s) const
{
  Eigen::VectorXd values(getConstraintDimension());

  size_t row = 0; 

  for(int i = 0; i < mConstraints.size(); ++i)
  {
    int dim = mConstraints.at(i)->getConstraintDimension();
    values.block(row, 0, dim, 1) = mConstraints.at(i)->getValue(_s);
    row = row + dim; 
  }

  return values;
}


//=============================================================================
state::JacobianPtr CompoundConstraint::getJacobian(
  const state::StatePtr& _s) const
{

  for(int i = 1; i < mConstraints.size(); ++i)
  {
    state::JacobianPtr jac2 = mConstraints.at(i)->getJacobian(_s); 
    jac = jac->append(jac2);
  }

  return jac;
}

//=============================================================================
std::vector<ConstraintType> CompoundConstraint::getConstraintTypes() const
{
  std::vector<ConstraintType> constraints;
  constraints.reserve(getConstraintDimension());

  for(int i = 0; i < mConstraints.size(); ++i)
  {
    std::vector<ConstraintType> constrs = mConstraints.at(i)->getConstraintTypes();
    constraints.insert( constraints.end(), constrs.begin(), constrs.end() );
  }

  return constraints;

}

}
}
