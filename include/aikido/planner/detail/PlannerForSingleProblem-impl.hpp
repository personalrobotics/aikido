#ifndef AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
#define AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_

#include "aikido/planner/PlannerForSingleProblem.hpp"

namespace aikido {
namespace planner {

//==============================================================================
template <typename Derived, typename ProblemT>
bool PlannerForSingleProblem<Derived, ProblemT>::canPlan(
    const Problem* problem) const
{
  return problem->getName() == ProblemT::getStaticName();
}

//==============================================================================
template <typename Derived, typename ProblemT>
std::unordered_set<std::string>
PlannerForSingleProblem<Derived, ProblemT>::getPlannableProblems() const
{
  return std::unordered_set<std::string>({ProblemT::getStaticName()});
}

//==============================================================================
template <typename Derived, typename ProblemT>
trajectory::TrajectoryPtr PlannerForSingleProblem<Derived, ProblemT>::plan(
    const Problem& problem, Result* result)
{
  if (!canPlan(&problem))
    return nullptr;

  return static_cast<Derived*>(this)->plan(
      static_cast<const typename Derived::TheProblem&>(problem),
      static_cast<typename Derived::TheResult*>(result));
}

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DETAIL_SINGLEPROBLEMPLANNER_IMPL_HPP_
