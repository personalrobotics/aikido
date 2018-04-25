#include "aikido/planner/PlannerForMultiProblem.hpp"

#include <cassert>

namespace aikido {
namespace planner {

//==============================================================================
bool PlannerForMultiProblem::canSolve(const Problem* problem) const
{
  auto& map = getPlanningFunctionMap();
  const auto search = map.find(problem->getName());

  if (search != map.end())
    return true;
  else
    return false;
}

//==============================================================================
trajectory::TrajectoryPtr PlannerForMultiProblem::plan(
    const Problem& problem, Result* result)
{
  auto& map = getPlanningFunctionMap();
  const auto search = map.find(problem.getName());

  if (search == map.end())
  {
    if (result)
    {
      // TODO(JS): not implemented
    }

    return nullptr;
  }

  assert(search->second);
  return search->second(&problem, result);
}

//==============================================================================
const PlannerForMultiProblem::PlanningFunctionMap&
PlannerForMultiProblem::getPlanningFunctionMap() const
{
  return const_cast<PlannerForMultiProblem*>(this)->getPlanningFunctionMap();
}

} // namespace planner
} // namespace aikido
