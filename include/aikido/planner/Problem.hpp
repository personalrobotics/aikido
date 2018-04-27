#ifndef AIKIDO_PLANNER_PROBLEM_HPP_
#define AIKIDO_PLANNER_PROBLEM_HPP_

#include <string>
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace planner {

/// Base class for various planning problems.
class Problem
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space that this problem associated with.
  explicit Problem(statespace::ConstStateSpacePtr stateSpace);

  /// Destructor.
  virtual ~Problem() = default;

  /// Returns type of this planning problem.
  virtual const std::string& getType() const = 0;

  /// Returns const state space.
  statespace::ConstStateSpacePtr getStateSpace() const;

protected:
  /// State space associated with this problem.
  statespace::ConstStateSpacePtr mStateSpace;
};
// Note: All the problem classes intentionally don't have setters. This is
// because we assume creating a new problem for different problem settings makes
// sense. We can revisit this design decision if we find a compelling reason to
// have setters.

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PROBLEM_HPP_
