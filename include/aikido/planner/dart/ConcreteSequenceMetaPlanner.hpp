#ifndef AIKIDO_PLANNER_CONCRETE_SEQUENCEMETAPLANNER_HPP_
#define AIKIDO_PLANNER_CONCRETE_SEQUENCEMETAPLANNER_HPP_

#include "aikido/planner/SequenceMetaPlanner.hpp"

#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {

/// Sequence meta planner with a default suite of planners suited to a wide
// array of planning tasks.
class ConcreteSequenceMetaPlanner : public SequenceMetaPlanner
{
public:
  /// Constructs a planner for the passed statespace.
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  ConcreteSequenceMetaPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton);

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) override;

  // Documentation inherited.
  std::shared_ptr<Planner> clone() const override;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONCRETE_SEQUENCEMETAPLANNER_HPP_
