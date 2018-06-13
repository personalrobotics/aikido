#ifndef AIKIDO_PLANNER_SEQUENCEMETAPLANNER_HPP_
#define AIKIDO_PLANNER_SEQUENCEMETAPLANNER_HPP_

#include "aikido/planner/CompositePlanner.hpp"

namespace aikido {
namespace planner {

/// A meta planner that solves a problem using the sub planners one-by-one
/// sequentially and returns the first successfully planned trajectory.
class SequenceMetaPlanner : public CompositePlanner
{
public:
  using CompositePlanner::CompositePlanner;

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) override;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_SEQUENCEMETAPLANNER_HPP_
