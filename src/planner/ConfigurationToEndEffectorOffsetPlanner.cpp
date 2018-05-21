#include "aikido/planner/ConfigurationToEndEffectorOffsetPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToEndEffectorOffsetPlanner::
    ConfigurationToEndEffectorOffsetPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace)
  : SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                         ConfigurationToEndEffectorOffset>(
        std::move(stateSpace))
{
  // Do nothing
}

//==============================================================================
statespace::dart::ConstMetaSkeletonStateSpacePtr
ConfigurationToEndEffectorOffsetPlanner::getMetaSkeletonStateSpace()
{
  return std::
      dynamic_pointer_cast<const statespace::dart::MetaSkeletonStateSpace>(
          mStateSpace);
}

} // namespace planner
} // namespace aikido
