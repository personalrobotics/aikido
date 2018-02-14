#include <aikido/constraint/Satisfied.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
Satisfied::Satisfied(statespace::StateSpacePtr _space)
  : mStateSpace(std::move(_space))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is null.");
}

//==============================================================================
statespace::StateSpacePtr Satisfied::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::size_t Satisfied::getConstraintDimension() const
{
  return 0;
}

//==============================================================================
std::vector<constraint::ConstraintType> Satisfied::getConstraintTypes() const
{
  return std::vector<constraint::ConstraintType>();
}

//==============================================================================
bool Satisfied::isSatisfied(
    const statespace::StateSpace::State* /*state*/,
    TestableOutcome* outcome) const
{
  auto defaultOutcomeObject
      = dynamic_cast_or_throw<DefaultTestableOutcome>(outcome);

  if (defaultOutcomeObject)
    defaultOutcomeObject->setSatisfiedFlag(true);
  return true;
}

//==============================================================================
std::unique_ptr<TestableOutcome> Satisfied::createOutcome() const
{
  return std::unique_ptr<TestableOutcome>(new DefaultTestableOutcome);
}

//==============================================================================
bool Satisfied::project(
    const statespace::StateSpace::State* _s,
    statespace::StateSpace::State* _out) const
{
  mStateSpace->copyState(_s, _out);
  return true;
}

//==============================================================================
void Satisfied::getValue(
    const statespace::StateSpace::State* /*_s*/, Eigen::VectorXd& _out) const
{
  _out = Eigen::Matrix<double, 0, 1>();
}

//==============================================================================
void Satisfied::getJacobian(
    const statespace::StateSpace::State* /*_s*/, Eigen::MatrixXd& _out) const
{
  _out = Eigen::Matrix<double, 0, 0>();
}

} // namespace constraint
} // namespace aikido
