#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp"
#include <dart/dynamics/dynamics.hpp>
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

#include <aikido/common/RNG.hpp>

using aikido::constraint::dart::InverseKinematicsSampleable;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::createSampleableBounds;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::InverseKinematics;
using ::dart::dynamics::ConstBodyNodePtr;
using ::dart::dynamics::BodyNodePtr;

using aikido::common::cloneRNGFrom;

namespace aikido {
namespace planner {
namespace dart {

//==============================================================================
ConfigurationToConfiguration_to_ConfigurationToTSR::
    ConfigurationToConfiguration_to_ConfigurationToTSR(
        std::shared_ptr<ConfigurationToConfigurationPlanner> planner,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton)
  : PlannerAdapter<ConfigurationToConfigurationPlanner,
                   ConfigurationToTSRPlanner>(
        std::move(planner), std::move(metaSkeleton))
{
  // Do nothing
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
  // TODO: Figure this wierd case with HERB out.
  aikido::common::RNGWrapper<std::mt19937> _rng
      = aikido::common::RNGWrapper<std::mt19937>(0);

  std::cout << "EE Node name IS: "
            << problem.getEndEffectorBodyNode()->getName() << std::endl;
  auto nonConstPtr
      = const_cast<BodyNode*>(problem.getEndEffectorBodyNode().get());
  BodyNodePtr endEffectorBodyNode(nonConstPtr);

  // auto matchingNodes = mMetaSkeleton->getBodyNodes(
  //     problem.getEndEffectorBodyNode()->getName());
  // if (matchingNodes.empty())
  //   throw std::invalid_argument(
  //       "End-effector BodyNode not found in Planner's MetaSkeleton.");
  // ::dart::dynamics::BodyNodePtr endEffectorBodyNode = matchingNodes.front();

  auto ik = InverseKinematics::create(endEffectorBodyNode);
  ik->setDofs(mMetaSkeleton->getDofs());

  // Convert TSR constraint into IK constraint
  InverseKinematicsSampleable ikSampleable(
      mMetaSkeletonStateSpace,
      mMetaSkeleton,
      std::const_pointer_cast<TSR>(problem.getGoalTSR()),
      createSampleableBounds(
          mMetaSkeletonStateSpace,
          std::move(cloneRNGFrom(_rng)[0])), // TODO: RNG should be in Planner
      ik,
      problem.getMaxSamples());
  auto generator = ikSampleable.createSampleGenerator();

  // NOTE: Const-casting should be removed once InverseKinematicsSampleable is
  // changed to take const constraints!
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

      // Set to start state
      mMetaSkeletonStateSpace->setState(
          mMetaSkeleton.get(), problem.getStartState());
    }

    // Create ConfigurationToConfiguration Problem.
    // NOTE: This is done here because the ConfigurationToConfiguration
    // problem stores a *cloned* scoped state of the passed state.
    auto delegateProblem = ConfigurationToConfiguration(
        mMetaSkeletonStateSpace,
        problem.getStartState(),
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
