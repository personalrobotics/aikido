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
PlannerPtr ConfigurationToConfiguration_to_ConfigurationToConfiguration::clone(
    common::RNG* rng) const
{
  throw std::runtime_error(
      "ConfigurationToConfiguration_to_Configuration should clone with metaSkeleton");
}

//==============================================================================
PlannerPtr ConfigurationToConfiguration_to_ConfigurationToConfiguration::clone(
    ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
    common::RNG* rng) const
{
  // TODO: assert metaSkeleton is a clone of mMetaSkeleton
  using aikido::planner::ConfigurationToConfigurationPlanner;

  auto castedDelegate = std::dynamic_pointer_cast<DartPlanner>(mDelegate);
  PlannerPtr clonedDelegate;

  if (castedDelegate)
    clonedDelegate = castedDelegate->clone(metaSkeleton, rng);
  else
    clonedDelegate = mDelegate->clone(rng);

  auto clonedCastedDelegate = std::dynamic_pointer_cast<
    ConfigurationToConfigurationPlanner>(clonedDelegate);

  if (!clonedCastedDelegate)
  {
    // GL: This shouldn't happen, so I think we can use static pointer cast above.
    throw std::runtime_error("Delegate has incorrect type.");
  }

  return std::make_shared<ConfigurationToConfiguration_to_ConfigurationToConfiguration>(
      clonedCastedDelegate,
      metaSkeleton);
}

//==============================================================================
bool ConfigurationToConfiguration_to_ConfigurationToConfiguration::stopPlanning()
{
  //  Does not allow stop planning
  return false;
}


} // namespace dart
} // namespace planner
} // namespace aikido
