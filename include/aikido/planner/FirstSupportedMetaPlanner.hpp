#ifndef AIKIDO_PLANNER_FIRSTSUPPORTEDMETAPLANNER_HPP_
#define AIKIDO_PLANNER_FIRSTSUPPORTEDMETAPLANNER_HPP_

#include "aikido/planner/CompositePlanner.hpp"

namespace aikido {
namespace planner {

class FirstSupportedMetaPlanner : public CompositePlanner
{
public:
  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) override;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_FIRSTSUPPORTEDMETAPLANNER_HPP_
