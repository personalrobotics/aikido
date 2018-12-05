#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToConfiguration.hpp"

#include <dart/dynamics/dynamics.hpp>
#include "aikido/planner/dart/util.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration_to_ConfigurationToConfiguration::
    ConfigurationToConfiguration_to_ConfigurationToConfiguration(
        std::shared_ptr<planner::ConfigurationToConfigurationPlanner> planner,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : PlannerAdapter<planner::ConfigurationToConfigurationPlanner,
                   planner::dart::ConfigurationToConfigurationPlanner>(
        std::move(planner), std::move(metaSkeleton))
{
  // Do nothing
}

//==============================================================================
trajectory::TrajectoryPtr
ConfigurationToConfiguration_to_ConfigurationToConfiguration::plan(
    const planner::dart::ConfigurationToConfiguration& problem,
    Planner::Result* result)
{
  // TODO: Check equality between state space of this planner and given problem.

  auto delegateProblem = planner::ConfigurationToConfiguration(
      mMetaSkeletonStateSpace,
      problem.getStartState(),
      problem.getGoalState(),
      problem.getConstraint());

  return mDelegate->plan(delegateProblem, result);
}

//==============================================================================
PlannerPtr ConfigurationToConfiguration_to_ConfigurationToConfiguration::clone() const
{
  using aikido::planner::ConfigurationToConfigurationPlanner;

  auto clonedDelegate = mDelegate->clone();
  auto clonedCastedDelegate = std::dynamic_pointer_cast<
    ConfigurationToConfigurationPlanner>(clonedDelegate);

  if (!clonedCastedDelegate)
  {
    // GL: This shouldn't happen, so I think we can use static pointer cast above.
    throw std::runtime_error("Delegate has incorrect type.");
  }

  // TODO: replace util::clone with DART's native clone method
  return std::make_shared<ConfigurationToConfiguration_to_ConfigurationToConfiguration>(
      clonedCastedDelegate,
      util::clone(mMetaSkeleton));
}

} // namespace dart
} // namespace planner
} // namespace aikido
