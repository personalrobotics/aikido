#ifndef AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
#define AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_

#include "aikido/planner/SingleProblemPlanner.hpp"

#include <utility>

namespace aikido {
namespace planner {

//==============================================================================
template <typename Derived, typename ProblemT>
SingleProblemPlanner<Derived, ProblemT>::SingleProblemPlanner(
    statespace::ConstStateSpacePtr stateSpace)
  : Planner(std::move(stateSpace))
{
  // Do nothing
}

//==============================================================================
template <typename Derived, typename ProblemT>
bool SingleProblemPlanner<Derived, ProblemT>::canSolve(
    const Problem* problem) const
{
  return problem->getType() == ProblemT::getStaticType();
}

//==============================================================================
template <typename Derived, typename ProblemT>
trajectory::TrajectoryPtr SingleProblemPlanner<Derived, ProblemT>::plan(
    const Problem& problem, Result* result)
{
  if (!canSolve(&problem))
    return nullptr;

  return static_cast<Derived*>(this)->plan(
      static_cast<const typename Derived::TheProblem&>(problem),
      static_cast<typename Derived::TheResult*>(result));
}

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
