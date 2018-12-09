#ifndef AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
#define AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_

#include "aikido/planner/SingleProblemPlanner.hpp"

#include <utility>

#include <iostream>

namespace aikido {
namespace planner {

//==============================================================================
template <typename Derived, typename ProblemT>
SingleProblemPlanner<Derived, ProblemT>::SingleProblemPlanner(
    statespace::ConstStateSpacePtr stateSpace, common::RNG* rng)
  : Planner(std::move(stateSpace), std::move(rng))
{
  // Do nothing
}

//==============================================================================
template <typename Derived, typename ProblemT>
bool SingleProblemPlanner<Derived, ProblemT>::canSolve(
    const Problem& problem) const
{
  std::cout << "Problem Type : " << problem.getType() << std::endl;
  if (problem.getType() == SolvableProblem::getStaticType())
    std::cout << "SingleProblemPlanner-impl: canSolve"<< std::endl;
  else
  {
    std::cout << "can't solve " << std::endl;
    std::cout << "SolvableProblem: " << SolvableProblem::getStaticType() << std::endl;
  }
  return problem.getType() == SolvableProblem::getStaticType();
}

//==============================================================================
template <typename Derived, typename ProblemT>
trajectory::TrajectoryPtr SingleProblemPlanner<Derived, ProblemT>::plan(
    const Problem& problem, Result* result)
{
  if (!canSolve(problem))
  {
    std::cout << "SingleProblemPlanner.plan Returning nullptr" << std::endl;
    return nullptr;
  }

  assert(dynamic_cast<const SolvableProblem*>(&problem));
#ifndef NDEBUG // Debug mode
  if (result)
  {
    assert(result && dynamic_cast<typename Derived::Result*>(result));
  }
#endif

  std::cout << "SingleProblemPlanner.plan calling plan" << std::endl;
  return static_cast<Derived*>(this)->plan(
      static_cast<const typename Derived::SolvableProblem&>(problem),
      static_cast<typename Derived::Result*>(result));
}

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
