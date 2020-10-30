#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToConfiguration.hpp"

#include <dart/dynamics/dynamics.hpp>

#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration_to_ConfigurationToConfiguration::
    ConfigurationToConfiguration_to_ConfigurationToConfiguration(
        std::shared_ptr<planner::ConfigurationToConfigurationPlanner> planner,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : PlannerAdapter<
        planner::ConfigurationToConfigurationPlanner,
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

  // NOTE: Make sure we lock the metaskeleton used to plan and return it to
  // correct state after.
  auto metaskeletonMutex = mMetaSkeleton->getLockableReference();
  std::lock_guard<::dart::common::LockableReference> lock(*metaskeletonMutex);
  // Save the current state of the space.
  auto saver = MetaSkeletonStateSaver(mMetaSkeleton);
  DART_UNUSED(saver);

  auto delegateProblem = planner::ConfigurationToConfiguration(
      mMetaSkeletonStateSpace,
      problem.getStartState(),
      problem.getGoalState(),
      problem.getConstraint());

  return mDelegate->plan(delegateProblem, result);
}

} // namespace dart
} // namespace planner
} // namespace aikido
