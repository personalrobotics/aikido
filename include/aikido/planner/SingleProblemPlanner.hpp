#ifndef AIKIDO_PLANNER_SINGLEPROBLEMPLANNER_HPP_
#define AIKIDO_PLANNER_SINGLEPROBLEMPLANNER_HPP_

#include "aikido/planner/Planner.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

/// SingleProblemPlanner is a base class for any concrete planner that are not
/// a CompositePlanner.
///
/// This is a curiously recurring template pattern implementation to avoid the
/// same implementation of canSolve() and plan() across concrete planners. By
/// inheriting this class the concrete planner doesn't need to implement those
/// virtual functions.
///
/// \tparam Derived Concrete planner type.
/// \tparam ProblemT Problem type the concrete planner is associated with.
template <typename Derived, typename ProblemT>
class SingleProblemPlanner : public Planner
{
public:
  using SolverbleProblem = ProblemT;

  /// Constructor
  ///
  /// \param[in] stateSpace State space associated with this planner.
  explicit SingleProblemPlanner(statespace::ConstStateSpacePtr stateSpace);

  // Documentation inherited.
  bool canSolve(const Problem& problem) const final override;

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) final override;
};

} // namespace planner
} // namespace aikido

#include "aikido/planner/detail/SingleProblemPlanner-impl.hpp"

#endif // AIKIDO_PLANNER_SINGLEPROBLEMPLANNER_HPP_
