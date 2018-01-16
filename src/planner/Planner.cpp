#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
bool Planner::canSolve(const Problem* problem)
{
  auto& map = getPlanningFunctionMap();
  const auto search = map.find(problem->getName());

  if (search != map.end())
    return true;
  else
    return false;
}

//==============================================================================
trajectory::TrajectoryPtr Planner::solve(
    const Problem* problem, Problem::Result* result)
{
  if (!problem)
  {
    if (result)
    {
      // TODO(JS): not implemented
    }

    return nullptr;
  }

  auto& map = getPlanningFunctionMap();
  const auto search = map.find(problem->getName());

  if (search == map.end())
  {
    if (result)
    {
      // TODO(JS): not implemented
    }

    return nullptr;
  }

  assert(search->second);
  return search->second(problem, result);
}

} // namespace planner
} // namespace aikido
