#ifndef AIKIDO_PLANNER_SINGLEPROBLEMPLANNER_HPP_
#define AIKIDO_PLANNER_SINGLEPROBLEMPLANNER_HPP_

#include <functional>
#include <unordered_map>
#include "aikido/planner/Planner.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

/// PlannerForSingleProblem is a base class for concrete planner classes that
/// only
/// support one planning problem.
template <typename Derived, typename ProblemT>
class PlannerForSingleProblem : public Planner
{
public:
  using TheProblem = ProblemT;
  using TheResult = Result;
  // TODO: Better naming

  // Documentation inherited.
  bool canPlan(const Problem* problem) const final override;

  // Documentation inherited.
  std::unordered_set<std::string> getPlannableProblems() const final override;

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) final override;
};

} // namespace planner
} // namespace aikido

#include "aikido/planner/detail/PlannerForSingleProblem-impl.hpp"

#endif // AIKIDO_PLANNER_SINGLEPROBLEMPLANNER_HPP_
