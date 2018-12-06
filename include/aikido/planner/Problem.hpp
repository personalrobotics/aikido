#ifndef AIKIDO_PLANNER_PROBLEM_HPP_
#define AIKIDO_PLANNER_PROBLEM_HPP_

#include <string>
#include "aikido/constraint/Testable.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace planner {

AIKIDO_DECLARE_POINTERS(Problem)

  /// Base class for various planning problems.
class Problem
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space that this problem associated with.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// If null is passed, constraint::Satisfied is created by default where
  /// constraint::Satisfied always returns true for the satisfaction query.
  /// \throw If \c stateSpace is null.
  /// \throw If The state space of \c constraint doesn't agree with
  /// \c statespace.
  Problem(
      statespace::ConstStateSpacePtr stateSpace,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Destructor.
  virtual ~Problem() = default;

  /// Returns type of this planning problem.
  virtual const std::string& getType() const = 0;

  /// Clones this planning problem.
  // virtual std::shared_ptr<Problem> clone() const = 0;

  /// Returns const state space.
  statespace::ConstStateSpacePtr getStateSpace() const;

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// State space associated with this problem.
  statespace::ConstStateSpacePtr mStateSpace;

  /// Trajectory-wide constraint that must be satisfied.
  constraint::ConstTestablePtr mConstraint;
};
// Note: All the problem classes intentionally don't have setters. This is
// because we assume creating a new problem for different problem settings makes
// sense. We can revisit this design decision if we find a compelling reason to
// have setters.

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PROBLEM_HPP_
