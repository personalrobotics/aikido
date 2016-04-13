#include <aikido/constraint/SatisfiedConstraint.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
SatisfiedConstraint::SatisfiedConstraint(statespace::StateSpacePtr _space)
  : mStateSpace(std::move(_space))
{
}

//=============================================================================
statespace::StateSpacePtr SatisfiedConstraint::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
size_t SatisfiedConstraint::getConstraintDimension() const
{
  return 0;
}

//=============================================================================
std::vector<constraint::ConstraintType> SatisfiedConstraint
  ::getConstraintTypes() const
{
  return std::vector<constraint::ConstraintType>();
}

//=============================================================================
bool SatisfiedConstraint::isSatisfied(
  const statespace::StateSpace::State* state) const
{
  return true;
}

//=============================================================================
bool SatisfiedConstraint::project(
  const statespace::StateSpace::State* _s,
  statespace::StateSpace::State* _out) const
{
  mStateSpace->copyState(_out, _s);
  return true;
}

//=============================================================================
Eigen::VectorXd SatisfiedConstraint::getValue(
  const statespace::StateSpace::State* _s) const
{
  return Eigen::Matrix<double, 0, 1>();
}

//=============================================================================
Eigen::MatrixXd SatisfiedConstraint::getJacobian(
  const statespace::StateSpace::State* _s) const
{
  return Eigen::Matrix<double, 0, 0>();
}

//=============================================================================
std::pair<Eigen::VectorXd, Eigen::MatrixXd> SatisfiedConstraint
  ::getValueAndJacobian(const statespace::StateSpace::State* _s) const
{
  return std::pair<Eigen::VectorXd, Eigen::MatrixXd>(
    Eigen::Matrix<double, 0, 1>(), Eigen::Matrix<double, 0, 0>());
}

} // namespace constraint
} // namespace aikido
