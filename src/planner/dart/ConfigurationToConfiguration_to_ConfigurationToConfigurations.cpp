#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToConfigurations.hpp"

#include <dart/dynamics/dynamics.hpp>

#include "aikido/common/RNG.hpp"
#include "aikido/distance/NominalConfigurationRanker.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

using aikido::distance::ConstConfigurationRankerPtr;
using aikido::distance::NominalConfigurationRanker;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::statespace::dart::MetaSkeletonStateSpace;

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration_to_ConfigurationToConfigurations::
    ConfigurationToConfiguration_to_ConfigurationToConfigurations(
        std::shared_ptr<planner::ConfigurationToConfigurationPlanner> planner,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
        distance::ConstConfigurationRankerPtr configurationRanker)
  : PlannerAdapter<
      planner::ConfigurationToConfigurationPlanner,
      planner::dart::ConfigurationToConfigurationsPlanner>(
      std::move(planner), std::move(metaSkeleton))
{
  mConfigurationRanker = std::move(configurationRanker);
}

//==============================================================================
trajectory::TrajectoryPtr
ConfigurationToConfiguration_to_ConfigurationToConfigurations::plan(
    const ConfigurationToConfigurations& problem, Planner::Result* result)
{
  // TODO: Check equality between state space of this planner and given problem.

  // NOTE: Make sure we lock the metaskeleton used to plan and return it to
  // correct state after.
  auto metaskeletonMutex = mMetaSkeleton->getLockableReference();
  std::lock_guard<::dart::common::LockableReference> lock(*metaskeletonMutex);
  // Save the current state of the space.
  auto saver = MetaSkeletonStateSaver(mMetaSkeleton);
  DART_UNUSED(saver);

  auto startState = problem.getStartState();

  // Use a ranker
  ConstConfigurationRankerPtr configurationRanker(mConfigurationRanker);
  if (!configurationRanker)
  {
    auto nominalState = mMetaSkeletonStateSpace->createState();
    mMetaSkeletonStateSpace->copyState(startState, nominalState);
    configurationRanker = std::make_shared<const NominalConfigurationRanker>(
        mMetaSkeletonStateSpace, mMetaSkeleton, nominalState);
  }

  std::vector<MetaSkeletonStateSpace::ScopedState> configurations;
  for (const auto goal : problem.getGoalStates())
    configurations.push_back(mMetaSkeletonStateSpace->cloneState(goal));
  if (configurations.empty())
    return nullptr;
  configurationRanker->rankConfigurations(configurations);

  for (std::size_t i = 0; i < configurations.size(); ++i)
  {
    // Create ConfigurationToConfiguration Problem.
    // NOTE: This is done here because the ConfigurationToConfiguration
    // problem stores a *cloned* scoped state of the passed state.
    auto delegateProblem = ConfigurationToConfiguration(
        mMetaSkeletonStateSpace,
        startState,
        configurations[i],
        problem.getConstraint());

    auto traj = mDelegate->plan(delegateProblem, result);
    if (traj)
      return traj;
  }

  return nullptr;
}

} // namespace dart
} // namespace planner
} // namespace aikido
