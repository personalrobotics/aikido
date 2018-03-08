#ifndef AIKIDO_PLANNER_RANKEDMETAPLANNER_HPP_
#define AIKIDO_PLANNER_RANKEDMETAPLANNER_HPP_

#include "aikido/planner/CompositePlanner.hpp"

namespace aikido {
namespace planner {

class RankedMetaPlanner : public CompositePlanner
{
public:
  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) override;

protected:
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_RANKEDMETAPLANNER_HPP_
