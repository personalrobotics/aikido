#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToConfiguration.hpp"

#include <dart/dynamics/dynamics.hpp>
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration_to_ConfigurationToConfiguration::
    ConfigurationToConfiguration_to_ConfigurationToConfiguration(
        std::shared_ptr<aikido::planner::ConfigurationToConfigurationPlanner>
            planner,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : PlannerAdapter<aikido::planner::ConfigurationToConfigurationPlanner,
                   aikido::planner::dart::ConfigurationToConfigurationPlanner>(
        std::move(planner), std::move(metaSkeleton))
{
  // Do nothing
}

//==============================================================================
trajectory::TrajectoryPtr
ConfigurationToConfiguration_to_ConfigurationToConfiguration::plan(
    const aikido::planner::dart::ConfigurationToConfiguration& problem,
    Planner::Result* result)
{
  // TODO: Check equality between state space of this planner and given problem.

  auto delegateProblem = aikido::planner::ConfigurationToConfiguration(
      mMetaSkeletonStateSpace,
      problem.getStartState(),
      problem.getGoalState(),
      problem.getConstraint());

  return mDelegate->plan(delegateProblem, result);
}

} // namespace dart
} // namespace planner
} // namespace aikido
