#ifndef AIKIDO_PLANNER_RANKEDMETAPLANNER_HPP_
#define AIKIDO_PLANNER_RANKEDMETAPLANNER_HPP_

#include "aikido/planner/MetaPlanner.hpp"

namespace aikido {
namespace planner {

class RankedMetaPlanner : public MetaPlanner
{
public:
  // Documentation inherited.
  trajectory::TrajectoryPtr solve(
      const Problem* problem, Problem::Result* result = nullptr) override;

protected:
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_RANKEDMETAPLANNER_HPP_
