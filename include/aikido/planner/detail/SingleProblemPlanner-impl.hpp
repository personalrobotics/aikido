#ifndef AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
#define AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_

#include "aikido/planner/SingleProblemPlanner.hpp"

#include <utility>

namespace aikido {
namespace planner {

//==============================================================================
template <typename Derived, typename ProblemT>
SingleProblemPlanner<Derived, ProblemT>::SingleProblemPlanner(
    statespace::ConstStateSpacePtr stateSpace, std::unique_ptr<common::RNG> rng)
  : Planner(std::move(stateSpace), std::move(rng))
{
  // Do nothing
}

//==============================================================================
template <typename Derived, typename ProblemT>
bool SingleProblemPlanner<Derived, ProblemT>::canSolve(
    const Problem& problem) const
{
  return problem.getType() == SolvableProblem::getStaticType();
}

//==============================================================================
template <typename Derived, typename ProblemT>
trajectory::TrajectoryPtr SingleProblemPlanner<Derived, ProblemT>::plan(
    const Problem& problem, Result* result)
{
  if (!canSolve(problem))
    return nullptr;

  assert(dynamic_cast<const SolvableProblem*>(&problem));
#ifndef NDEBUG // Debug mode
  if (result)
  {
    assert(result && dynamic_cast<typename Derived::Result*>(result));
  }
#endif

  return static_cast<Derived*>(this)->plan(
      static_cast<const typename Derived::SolvableProblem&>(problem),
      static_cast<typename Derived::Result*>(result));
}

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
