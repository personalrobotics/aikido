#ifndef AIKIDO_PLANNER_PLANNER_HPP_
#define AIKIDO_PLANNER_PLANNER_HPP_

#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include "aikido/planner/Problem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

/// Base class for a meta-planner
class Planner
{
public:
  virtual bool canSolve(const Problem* problem);

  virtual trajectory::TrajectoryPtr solve(
      const Problem* problem, Problem::Result* result = nullptr);

protected:
  using PlanningFunction = std::function<trajectory::TrajectoryPtr(
      const Problem*, Problem::Result*)>;
  using PlanningFunctionMap = std::map<std::string, PlanningFunction>;

  virtual PlanningFunctionMap& getPlanningFunctionMap() = 0;

  template <class ProblemT, typename T, typename R, typename... Args>
  void registerPlanningFunction(R (T::*func)(Args...));
};

} // namespace planner
} // namespace aikido

#include "aikido/planner/detail/Planner-impl.hpp"

#endif // AIKIDO_PLANNER_PLANNER_HPP_
