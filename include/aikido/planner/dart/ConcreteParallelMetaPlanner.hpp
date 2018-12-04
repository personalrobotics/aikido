#ifndef AIKIDO_PLANNER_CONCRETE_PARALLELMETAPLANNER_HPP_
#define AIKIDO_PLANNER_CONCRETE_PARALLELMETAPLANNER_HPP_

#include <future>
#include <mutex>

#include "aikido/planner/ParallelMetaPlanner.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {

/// Parallel meta planner with a default suite of planners suited to a wide
// array of planning tasks.
class ConcreteParallelMetaPlanner : public ParallelMetaPlanner
{
public:
  /// Constructs a planner for the passed statespace.
  ///
  /// \param[in] stateSpace State space associated with this planner.
  /// \param[in] metaSkeleton Metaskeleton associated with this planner.
  /// \param[in] planners Set of planners for this planner to run in parallel.
  ConcreteParallelMetaPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      const std::vector<PlannerPtr>& planners = std::vector<PlannerPtr>());

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) override;

private:
  // True when planning
  std::atomic_bool mRunning;

  // Protects mRunning, mPromises
  std::mutex mMutex;

  statespace::dart::ConstMetaSkeletonStateSpacePtr mStateSpace;
  std::vector<PlannerPtr> mPlanners;
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONCRETE_PARALLELMETAPLANNER_HPP_
