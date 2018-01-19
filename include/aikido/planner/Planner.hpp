#ifndef AIKIDO_PLANNER_PLANNER_HPP_
#define AIKIDO_PLANNER_PLANNER_HPP_

#include <functional>
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
  /// Returns true if this planner can solve \c problem.
  virtual bool canSolve(const Problem* problem);

  /// Solves \c problem returning the result to \c result.
  virtual trajectory::TrajectoryPtr solve(
      const Problem* problem, Problem::Result* result = nullptr);

protected:
  using PlanningFunction = std::function<trajectory::TrajectoryPtr(
      const Problem*, Problem::Result*)>;
  using PlanningFunctionMap = std::unordered_map<std::string, PlanningFunction>;

  /// Returns planning function map.
  ///
  /// The concrete planner classes should contain a static map and return it by
  /// overriding this function.
  virtual PlanningFunctionMap& getPlanningFunctionMap() = 0;

  /// Registers planning function to this planner.
  template <class ProblemT, typename T, typename R, typename... Args>
  void registerPlanningFunction(R (T::*func)(Args...));
};

} // namespace planner
} // namespace aikido

#include "aikido/planner/detail/Planner-impl.hpp"

#endif // AIKIDO_PLANNER_PLANNER_HPP_
