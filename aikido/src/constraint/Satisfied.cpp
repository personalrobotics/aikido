#include <aikido/constraint/Satisfied.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
Satisfied::Satisfied(statespace::StateSpacePtr _space)
  : mStateSpace(std::move(_space))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is null.");
}

//=============================================================================
statespace::StateSpacePtr Satisfied::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
size_t Satisfied::getConstraintDimension() const
{
  return 0;
}

//=============================================================================
std::vector<constraint::ConstraintType> Satisfied
  ::getConstraintTypes() const
{
  return std::vector<constraint::ConstraintType>();
}

//=============================================================================
bool Satisfied::isSatisfied(
  const statespace::StateSpace::State* state) const
{
  return true;
}

//=============================================================================
bool Satisfied::project(
  const statespace::StateSpace::State* _s,
  statespace::StateSpace::State* _out) const
{
  mStateSpace->copyState(_s, _out);
  return true;
}

//=============================================================================
Eigen::VectorXd Satisfied::getValue(
  const statespace::StateSpace::State* _s) const
{
  return Eigen::Matrix<double, 0, 1>();
}

//=============================================================================
Eigen::MatrixXd Satisfied::getJacobian(
  const statespace::StateSpace::State* _s) const
{
  return Eigen::Matrix<double, 0, 0>();
}

//=============================================================================
std::pair<Eigen::VectorXd, Eigen::MatrixXd> Satisfied
  ::getValueAndJacobian(const statespace::StateSpace::State* _s) const
{
  return std::pair<Eigen::VectorXd, Eigen::MatrixXd>(
    Eigen::Matrix<double, 0, 1>(), Eigen::Matrix<double, 0, 0>());
}

} // namespace constraint
} // namespace aikido
