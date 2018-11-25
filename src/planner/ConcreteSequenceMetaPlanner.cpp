#include "aikido/planner/ConcreteSequenceMetaPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConcreteSequenceMetaPlanner::ConcreteSequenceMetaPlanner(
    statespace::ConstStateSpacePtr stateSpace)
  : SequenceMetaPlanner(std::move(stateSpace), std::vector<PlannerPtr>())
{
  // TODO.
}

} // namespace planner
} // namespace aikido
