#ifndef AIKIDO_PLANNER_SEQUENCEMETAPLANNER_HPP_
#define AIKIDO_PLANNER_SEQUENCEMETAPLANNER_HPP_

#include "aikido/planner/CompositePlanner.hpp"

namespace aikido {
namespace planner {

class SequenceMetaPlanner : public CompositePlanner
{
public:
  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) override;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_SEQUENCEMETAPLANNER_HPP_
