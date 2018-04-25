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

  /// Returns name.
  virtual const std::string& getName() const = 0;

  /// Returns const state space.
  statespace::ConstStateSpacePtr getStateSpace() const;

protected:
  /// State space associated with this problem.
  statespace::ConstStateSpacePtr mStateSpace;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PROBLEM_HPP_
