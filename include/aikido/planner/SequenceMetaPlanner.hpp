#ifndef AIKIDO_PLANNER_SEQUENCEMETAPLANNER_HPP_
#define AIKIDO_PLANNER_SEQUENCEMETAPLANNER_HPP_

#include "aikido/planner/MetaPlanner.hpp"

namespace aikido {
namespace planner {

class SequenceMetaPlanner : public MetaPlanner
{
public:
  // Documentation inherited.
  trajectory::TrajectoryPtr solve(
      const Problem* problem, Problem::Result* result = nullptr) override;

protected:
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_SEQUENCEMETAPLANNER_HPP_
