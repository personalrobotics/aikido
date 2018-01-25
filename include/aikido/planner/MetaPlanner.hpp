#ifndef AIKIDO_PLANNER_METAPLANNER_HPP_
#define AIKIDO_PLANNER_METAPLANNER_HPP_

#include <unordered_set>
#include <vector>
#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace planner {

class MetaPlanner : public Planner
{
public:
  /// Constructor.
  explicit MetaPlanner(const std::vector<PlannerPtr>& planners);

  /// Adds planner.
  void addPlanner(PlannerPtr planner);

  /// Returns true if this MetaPlanner contains \c planner.
  bool hasPlanner(const Planner* planner);

  /// Returns all the planners in this MetaPlanner.
  const std::vector<PlannerPtr>& getPlanners() const;

  /// Returns planner given \c index.
  ///
  /// \throw If index is equal to or greater than the number of planners in this
  /// MetaPlanner.
  PlannerPtr getPlanner(std::size_t index);

  // Documentation inherited.
  bool canSolve(const Problem* problem) const override;

  // Documentation inherited.
  std::unordered_set<std::string> getSolvableProblems() const override;

  /// Returns set of planners that can solve \c problem.
  std::unordered_set<PlannerPtr> getPlannersCanSolve(
      const Problem* problem) const;

protected:
  /// Planners
  std::vector<PlannerPtr> mPlanners;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_METAPLANNER_HPP_
