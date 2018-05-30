#include "aikido/planner/dart/ConfigurationToEndEffectorOffsetPlanner.hpp"

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToEndEffectorOffsetPlanner::
    ConfigurationToEndEffectorOffsetPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace)
  : SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                         ConfigurationToEndEffectorOffset>(stateSpace)
  , mMetaSkeletonStateSpace(std::move(stateSpace))
{
  // Do nothing
}

//==============================================================================
statespace::dart::ConstMetaSkeletonStateSpacePtr
ConfigurationToEndEffectorOffsetPlanner::getMetaSkeletonStateSpace()
{
  return mMetaSkeletonStateSpace;
}

} // namespace dart
} // namespace planner
} // namespace aikido
