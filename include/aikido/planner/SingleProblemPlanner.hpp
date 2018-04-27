#ifndef AIKIDO_PLANNER_SINGLEPROBLEMPLANNER_HPP_
#define AIKIDO_PLANNER_SINGLEPROBLEMPLANNER_HPP_

#include "aikido/planner/Planner.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

/// SingleProblemPlanner is a base class for concrete planner classes that
/// only support one planning problem.
// TODO(JS): Add docstring for template arguments.
template <typename Derived, typename ProblemT>
class SingleProblemPlanner : public Planner
{
public:
  using TheProblem = ProblemT;
  using TheResult = Result;
  // TODO: Better naming

  /// Constructor
  ///
  /// \param[in] stateSpace State space associated with this planner.
  explicit SingleProblemPlanner(statespace::ConstStateSpacePtr stateSpace);

  // Documentation inherited.
  bool canSolve(const Problem* problem) const final override;

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) final override;
};

} // namespace planner
} // namespace aikido

#include "aikido/planner/detail/SingleProblemPlanner-impl.hpp"

#endif // AIKIDO_PLANNER_SINGLEPROBLEMPLANNER_HPP_
