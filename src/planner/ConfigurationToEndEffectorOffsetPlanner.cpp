#include "aikido/planner/ConfigurationToEndEffectorOffsetPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
ConfigurationToEndEffectorOffsetPlanner::ConfigurationToEndEffectorOffsetPlanner(
    statespace::ConstStateSpacePtr stateSpace)
  : SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                         ConfigurationToEndEffectorOffset>(std::move(stateSpace))
{
  // Do nothing
}

} // namespace planner
} // namespace aikido
