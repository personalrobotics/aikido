#include "aikido/planner/Problem.hpp"

#include "aikido/constraint/Satisfied.hpp"

namespace aikido {
namespace planner {

//==============================================================================
Problem::Problem(
    statespace::ConstStateSpacePtr stateSpace,
    constraint::ConstTestablePtr constraint)
  : mStateSpace(std::move(stateSpace)), mConstraint(std::move(constraint))
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace shouldn't be nullptr.");

  if (!mConstraint)
  {
    mConstraint = std::make_shared<constraint::Satisfied>(
        std::const_pointer_cast<statespace::StateSpace>(mStateSpace));
    // TODO(JS): Remove this const-cast once we fix const-correctness of
    // constraint classes
  }

  if (mStateSpace != mConstraint->getStateSpace())
  {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
}

//==============================================================================
statespace::ConstStateSpacePtr Problem::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
constraint::ConstTestablePtr Problem::getConstraint() const
{
  return mConstraint;
}

} // namespace planner
} // namespace aikido
