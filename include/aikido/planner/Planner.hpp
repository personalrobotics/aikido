#ifndef AIKIDO_PLANNER_PLANNER_HPP_
#define AIKIDO_PLANNER_PLANNER_HPP_

#include <functional>
#include <unordered_set>
#include "aikido/planner/Problem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

/// Base class for a meta-planner
class Planner
{
public:
  /// Returns true if this planner can solve \c problem.
  virtual bool canSolve(const Problem* problem) = 0;

  /// Returns all the problems that this planner can solve.
  virtual std::unordered_set<std::string> getSolvableProblems() const = 0;

  /// Solves \c problem returning the result to \c result.
  virtual trajectory::TrajectoryPtr solve(
      const Problem* problem, Problem::Result* result = nullptr)
      = 0;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANNER_HPP_
