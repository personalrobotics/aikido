#ifndef AIKIDO_PLANNER_CONCRETE_PARALLELMETAPLANNER_HPP_
#define AIKIDO_PLANNER_CONCRETE_PARALLELMETAPLANNER_HPP_

#include <future>
#include <mutex>

//#include "aikido/common/pointers.hpp"
#include "aikido/planner/ParallelMetaPlanner.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Parallel meta planner with a default suite of planners suited to a wide
// array of planning tasks.
class ConcreteParallelMetaPlanner : public ParallelMetaPlanner
{
public:
  /// Constructs a parallel planner for the passed statespace and planners.
  ///
  /// \param[in] stateSpace State space associated with this planner.
  /// \param[in] metaSkeleton Metaskeleton associated with this planner.
  /// \param[in] planners Set of planners for this planner to run in parallel.
  ConcreteParallelMetaPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      const std::vector<PlannerPtr>&
      planners = std::vector<PlannerPtr>());

  /// Constructs a parallel planner by cloning the passed planner.
  ///
  /// \param[in] stateSpace State space associated with this planner.
  /// \param[in] metaSkeleton Metaskeleton associated with this planner.
  /// \param[in] planner Planner to clone.
  /// \param[in] numCopies Number of clones to make
  /// \param[in] rngs RNGs to pass to each cloned planner. If nonempty,
  /// the number of rngs should match numCopies.
  ConcreteParallelMetaPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      const PlannerPtr& planner,
      std::size_t numCopies = 1,
      const std::vector<common::RNG*> rngs =
        std::vector<common::RNG*>());

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) override;

  // Documentation inherited.
  virtual PlannerPtr clone(common::RNG* rng = nullptr) const override;

private:
  // True when planning
  std::atomic_bool mRunning;

  // Protects mRunning
  std::mutex mMutex;

  statespace::dart::ConstMetaSkeletonStateSpacePtr mStateSpace;
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONCRETE_PARALLELMETAPLANNER_HPP_
