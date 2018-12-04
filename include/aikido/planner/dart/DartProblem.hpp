#ifndef AIKIDO_PLANNER_DARTPROBLEM_HPP_
#define AIKIDO_PLANNER_DARTPROBLEM_HPP_

#include <dart/dart.hpp>
#include "aikido/planner/Problem.hpp"

namespace aikido {
namespace planner {

AIKIDO_DECLARE_POINTERS(DartProblem)

  /// Base class for various planning problems with metaSkeleton.
class DartProblem : public Problem
{
public:
  /// Constructor.
  /// Gets passed to Problem constructor.
  /// \param[in] stateSpace State space that this problem associated with.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// If null is passed, constraint::Satisfied is created by default where
  /// constraint::Satisfied always returns true for the satisfaction query.
  /// \throw If \c stateSpace is null.
  /// \throw If The state space of \c constraint doesn't agree with
  /// \c statespace.
  DartProblem(
      statespace::ConstStateSpacePtr stateSpace,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Clones this planning problem with a new metaskeleton
  /// \param[in] metaSkeleton Metaskeleton with which to clone the problem
  virtual std::shared_ptr<Problem> clone(
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton) const = 0;
};
// Note: All the problem classes intentionally don't have setters. This is
// because we assume creating a new problem for different problem settings makes
// sense. We can revisit this design decision if we find a compelling reason to
// have setters.

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DARTPROBLEM_HPP_
