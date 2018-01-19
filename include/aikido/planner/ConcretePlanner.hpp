#ifndef AIKIDO_PLANNER_CONCRETEPLANNER_HPP_
#define AIKIDO_PLANNER_CONCRETEPLANNER_HPP_

#include <functional>
#include <unordered_map>
#include "aikido/planner/Planner.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

class ConcretePlanner : public Planner
{
public:
  // Documentation inherited.
  bool canSolve(const Problem* problem) override;

  // Documentation inherited.
  std::unordered_set<std::string> getSolvableProblems() const override;

  // Documentation inherited.
  trajectory::TrajectoryPtr solve(
      const Problem* problem, Problem::Result* result = nullptr) override;

protected:
  using PlanningFunction = std::function<trajectory::TrajectoryPtr(
      const Problem*, Problem::Result*)>;
  using PlanningFunctionMap = std::unordered_map<std::string, PlanningFunction>;

  /// Returns planning function map.
  ///
  /// The concrete planner classes should contain a static map and return it by
  /// overriding this function.
  virtual PlanningFunctionMap& getPlanningFunctionMap() = 0;

  /// Returns planning function map.
  const PlanningFunctionMap& getPlanningFunctionMap() const;

  /// Registers planning function to this planner.
  template <class ProblemT, typename T, typename R, typename... Args>
  void registerPlanningFunction(R (T::*func)(Args...));
};

} // namespace planner
} // namespace aikido

#include "aikido/planner/detail/ConcretePlanner-impl.hpp"

#endif // AIKIDO_PLANNER_CONCRETEPLANNER_HPP_
