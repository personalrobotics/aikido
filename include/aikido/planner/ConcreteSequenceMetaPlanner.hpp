#ifndef AIKIDO_PLANNER_CONCRETE_SEQUENCEMETAPLANNER_HPP_
#define AIKIDO_PLANNER_CONCRETE_SEQUENCEMETAPLANNER_HPP_

#include "aikido/planner/SequenceMetaPlanner.hpp"

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
  ConcreteSequenceMetaPlanner(statespace::ConstStateSpacePtr stateSpace);

  // Documentation inherited.
  trajectory::TrajectoryPtr plan(
      const Problem& problem, Result* result = nullptr) override;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONCRETE_SEQUENCEMETAPLANNER_HPP_
