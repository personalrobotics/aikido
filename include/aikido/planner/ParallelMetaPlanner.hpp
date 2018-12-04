#ifndef AIKIDO_PLANNER_PARALLELMETAPLANNER_HPP_
#define AIKIDO_PLANNER_PARALLELMETAPLANNER_HPP_

#include "aikido/planner/CompositePlanner.hpp"

namespace aikido {
namespace planner {

/// A meta planner that solves a problem using the sub planners in parallel
/// and returns the first successfully planned trajectory.
class ParallelMetaPlanner : public CompositePlanner
{
public:
  using CompositePlanner::CompositePlanner;

  // Documentation inherited.
  virtual trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) = 0;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PARALLELMETAPLANNER_HPP_
