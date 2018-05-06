#ifndef AIKIDO_PLANNER_COMPOSITEPLANNER_HPP_
#define AIKIDO_PLANNER_COMPOSITEPLANNER_HPP_

#include <vector>

#include "aikido/statespace/StateSpace.hpp"
#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace planner {

AIKIDO_DECLARE_POINTERS(CompositePlanner)

/// CompositePlanner is a base class for concrete planner classes that contain
/// multiple planners.
class CompositePlanner : public Planner
{
public:
  /// Constructs given list of planners.
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] planners Planners that this CompositePlanner will contain.
  CompositePlanner(
      statespace::ConstStateSpacePtr stateSpace,
      const std::vector<PlannerPtr>& planners = std::vector<PlannerPtr>());

  /// Returns true if this CompositePlanner contains \c planner.
  bool hasPlanner(const Planner* planner) const;

  // Documentation inherited.
  bool canSolve(const Problem& problem) const override;

protected:
  /// Planners.
  const std::vector<PlannerPtr> mPlanners;
  // We use std::vector to keep the order of planners. This is because some
  // concrete planners rely on the order. Alternatively, we could remove
  // mPlanners from here and let the concrete classes maintain the planner
  // containers accordingly. In that case, we should also remove getPlanner and
  // make addPlanner/hasPlanner/getPlanners as pure virtual functions.
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_COMPOSITEPLANNER_HPP_
