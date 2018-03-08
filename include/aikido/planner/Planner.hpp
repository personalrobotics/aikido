#ifndef AIKIDO_PLANNER_PLANNER_HPP_
#define AIKIDO_PLANNER_PLANNER_HPP_

#include <functional>
#include <unordered_set>
#include "aikido/common/pointers.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

AIKIDO_DECLARE_POINTERS(Planner)

/// Base class for a meta-planner
class Planner
{
public:
  class Result;

  /// Returns true if this planner can solve \c problem.
  virtual bool canPlan(const Problem* problem) const = 0;
  // TODO: Change parameter type to const reference

  /// Returns all the problems that this planner can solve.
  virtual std::unordered_set<std::string> getPlannableProblems() const = 0;

  /// Solves \c problem returning the result to \c result.
  virtual trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr)
      = 0;
};

/// Base class for planning result of various planning problems.
class Planner::Result
{
public:
  /// Destructor.
  virtual ~Result() = default;

  /// Sets message.
  void setMessage(const std::string& message);

  /// Returns message.
  const std::string& getMessage() const;

protected:
  /// Message.
  std::string mMessage;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANNER_HPP_
