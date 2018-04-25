#ifndef AIKIDO_PLANNER_COMPOSITEPLANNER_HPP_
#define AIKIDO_PLANNER_COMPOSITEPLANNER_HPP_

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace planner {

/// CompositePlanner is a base class for concrete planner classes that contain
/// multiple planners.
class CompositePlanner : public Planner
{
public:
  /// Constructor.
  CompositePlanner();

  /// Constructs given list of planners.
  explicit CompositePlanner(const std::vector<PlannerPtr>& planners);

  /// Adds planner.
  void addPlanner(PlannerPtr planner);

  /// Returns true if this CompositePlanner contains \c planner.
  bool hasPlanner(const Planner* planner);

  /// Returns all the planners in this CompositePlanner.
  const std::vector<PlannerPtr>& getPlanners() const;

  /// Returns planner given \c index.
  ///
  /// \throw If index is equal to or greater than the number of planners in this
  /// CompositePlanner.
  PlannerPtr getPlanner(std::size_t index);

  // Documentation inherited.
  virtual bool canPlan(const Problem* problem) const override;

protected:
  /// Planners.
  std::vector<PlannerPtr> mPlanners;
  // We use std::vector to keep the order of planners. This is because some
  // concrete rely on the order. Alternatively, we could remove mPlanners from
  // here and let the concrete classes maintain the planner containers
  // accordingly. In that case, we should also remove getPlanner and make
  // addPlanner/hasPlanner/getPlanners as pure virtual functions.
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_COMPOSITEPLANNER_HPP_
