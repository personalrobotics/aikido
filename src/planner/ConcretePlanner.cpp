#include "aikido/planner/ConcretePlanner.hpp"

#include <cassert>

namespace aikido {
namespace planner {

//==============================================================================
bool ConcretePlanner::canSolve(const Problem* problem) const
{
  auto& map = getPlanningFunctionMap();
  const auto search = map.find(problem->getName());

  if (search != map.end())
    return true;
  else
    return false;
}

//==============================================================================
std::unordered_set<std::string> ConcretePlanner::getSolvableProblems() const
{
  auto& map = getPlanningFunctionMap();

  std::unordered_set<std::string> problems;
  problems.reserve(map.size());

  for (auto& item : map)
    problems.insert(item.first);

  return problems;
}

//==============================================================================
trajectory::TrajectoryPtr ConcretePlanner::solve(
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

//==============================================================================
const ConcretePlanner::PlanningFunctionMap&
ConcretePlanner::getPlanningFunctionMap() const
{
  return const_cast<ConcretePlanner*>(this)->getPlanningFunctionMap();
}

} // namespace planner
} // namespace aikido
