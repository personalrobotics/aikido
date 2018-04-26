#ifndef AIKIDO_PLANNER_MULTIPROBLEMPLANNER_HPP_
#define AIKIDO_PLANNER_MULTIPROBLEMPLANNER_HPP_

#include <functional>
#include <unordered_map>
#include "aikido/planner/Planner.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

/// PlannerForMultiProblem is a base class for concrete planner classes that
/// support multiple planning problems.
class PlannerForMultiProblem : public Planner
{
public:
  // Documentation inherited.
  bool canSolve(const Problem* problem) const final override;

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) final override;

protected:
  using PlanningFunction
      = std::function<trajectory::TrajectoryPtr(const Problem*, Result*)>;
  using PlanningFunctionMap = std::unordered_map<std::string, PlanningFunction>;

  /// Returns planning function map.
  ///
  /// The concrete planner classes should contain a static map and return it by
  /// overriding this function.
  virtual PlanningFunctionMap& getPlanningFunctionMap() = 0;

  /// Returns planning function map.
  const PlanningFunctionMap& getPlanningFunctionMap() const;

  /// Registers planning function to this planner.
  // TODO(JS): Add docstring for template parameter.
  template <class ProblemT, typename T, typename R, typename... Args>
  void registerPlanningFunction(R (T::*func)(Args...));
};

} // namespace planner
} // namespace aikido

#include "aikido/planner/detail/PlannerForMultiProblem-impl.hpp"

#endif // AIKIDO_PLANNER_MULTIPROBLEMPLANNER_HPP_
