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
    const ConfigurationToConfiguration& problem, Planner::Result* result)
{
  // TODO: Check equality between state space of this planner and given problem.

  // Get the start state form the MetaSkeleton, since this is a DART planner.
  auto startState = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), startState);

  auto delegateProblem = ConfigurationToConfiguration(
      mMetaSkeletonStateSpace,
      startState,
      problem.getGoalState(),
      problem.getConstraint());

  return mDelegate->plan(delegateProblem, result);
}

} // namespace dart
} // namespace planner
} // namespace aikido
