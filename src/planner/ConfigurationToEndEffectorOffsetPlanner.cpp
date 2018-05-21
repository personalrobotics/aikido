#include "aikido/planner/ConfigurationToEndEffectorOffsetPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToEndEffectorOffsetPlanner::
    ConfigurationToEndEffectorOffsetPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace)
  : SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                         ConfigurationToEndEffectorOffset>(stateSpace)
  , mMetaSkeletonStateSpace(stateSpace)
{
  // Do nothing
}

//==============================================================================
statespace::dart::ConstMetaSkeletonStateSpacePtr
ConfigurationToEndEffectorOffsetPlanner::getMetaSkeletonStateSpace()
{
  return mMetaSkeletonStateSpace;
}

} // namespace planner
} // namespace aikido
