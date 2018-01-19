#ifndef AIKIDO_PLANNER_METAPLANNER_HPP_
#define AIKIDO_PLANNER_METAPLANNER_HPP_

#include <vector>
#include "aikido/planner/Planner.hpp"
#include "aikido/planner/smart_pointer.hpp"

namespace aikido {
namespace planner {

class MetaPlanner : public Planner
{
public:
  // Documentation inherited.
  bool canSolve(const Problem* problem) override;

  // Documentation inherited.
  std::unordered_set<std::string> getSolvableProblems() const override;

  /// Returns set of planners that can solve \c problem.
  std::unordered_set<PlannerPtr> getPlannersCanSolve(
      const Problem* problem) const;

protected:
  std::vector<PlannerPtr> mPlanners;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_METAPLANNER_HPP_
