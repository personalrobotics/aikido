#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp"

#include <dart/dynamics/dynamics.hpp>
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

using aikido::common::cloneRNGFrom;
using aikido::constraint::dart::InverseKinematicsSampleable;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::createSampleableBounds;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::InverseKinematics;
using ::dart::dynamics::ConstBodyNodePtr;
using ::dart::dynamics::BodyNodePtr;

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration_to_ConfigurationToTSR::
    ConfigurationToConfiguration_to_ConfigurationToTSR(
        std::shared_ptr<planner::ConfigurationToConfigurationPlanner> planner,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
        distance::ConfigurationRankerPtr configurationRanker)
  : PlannerAdapter<planner::ConfigurationToConfigurationPlanner,
                   ConfigurationToTSRPlanner>(
        std::move(planner), std::move(metaSkeleton))
{
  mConfigurationRanker = std::move(configurationRanker);
}

//==============================================================================
trajectory::TrajectoryPtr
ConfigurationToConfiguration_to_ConfigurationToTSR::plan(
    const ConfigurationToTSR& problem, Planner::Result* result)
{
  // TODO: Check equality between state space of this planner and given problem.

  // TODO: DART may be updated to check for single skeleton
  if (mMetaSkeleton->getNumDofs() == 0)
    throw std::invalid_argument("MetaSkeleton has 0 degrees of freedom.");

  auto skeleton = mMetaSkeleton->getDof(0)->getSkeleton();
  for (std::size_t i = 1; i < mMetaSkeleton->getNumDofs(); ++i)
  {
    if (mMetaSkeleton->getDof(i)->getSkeleton() != skeleton)
      throw std::invalid_argument("MetaSkeleton has more than 1 skeleton.");
  }

  // Create an IK solver with MetaSkeleton DOFs
  auto matchingNodes
      = skeleton->getBodyNodes(problem.getEndEffectorBodyNode()->getName());
  if (matchingNodes.empty())
    throw std::invalid_argument(
        "End-effector BodyNode not found in Planner's MetaSkeleton.");
  ::dart::dynamics::BodyNodePtr endEffectorBodyNode = matchingNodes.front();

  auto ik = InverseKinematics::create(endEffectorBodyNode);
  ik->setDofs(mMetaSkeleton->getDofs());

  // Get the start state from the MetaSkeleton, since this is a DART planner.
  auto startState = mMetaSkeletonStateSpace->createState();
  mMetaSkeletonStateSpace->getState(mMetaSkeleton.get(), startState);

  auto rng = std::move(cloneRNGFrom(*mDelegate->getRng())[0]);
  // Convert TSR constraint into IK constraint.
  // NOTE: Const-casting should be removed once InverseKinematicsSampleable is
  // changed to take const constraints!
  InverseKinematicsSampleable ikSampleable(
      mMetaSkeletonStateSpace,
      mMetaSkeleton,
      std::const_pointer_cast<TSR>(problem.getGoalTSR()),
      createSampleableBounds(mMetaSkeletonStateSpace, std::move(rng)),
      ik,
      problem.getMaxSamples());
  auto generator = ikSampleable.createSampleGenerator();

  auto saver = MetaSkeletonStateSaver(mMetaSkeleton);
  DART_UNUSED(saver);

  auto goalState = mMetaSkeletonStateSpace->createState();
  while (generator->canSample())
  {
    // Sample from TSR
    {
      std::lock_guard<std::mutex> lock(skeleton->getMutex());
      bool sampled = generator->sample(goalState);
      if (!sampled)
        continue;
    }

    // Create ConfigurationToConfiguration Problem.
    // NOTE: This is done here because the ConfigurationToConfiguration
    // problem stores a *cloned* scoped state of the passed state.
    auto delegateProblem = ConfigurationToConfiguration(
        mMetaSkeletonStateSpace,
        startState,
        goalState,
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
