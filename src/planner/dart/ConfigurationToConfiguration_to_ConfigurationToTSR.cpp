#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp"
#include <dart/dynamics/dynamics.hpp>
#include "aikido/constraint/FiniteSampleable.hpp"
#include "aikido/constraint/SequentialSampleable.hpp"
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/distance/NominalConfigurationRanker.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

using aikido::constraint::FiniteSampleable;
using aikido::constraint::SequentialSampleable;
using aikido::constraint::dart::InverseKinematicsSampleable;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::createSampleableBounds;
using aikido::distance::NominalConfigurationRanker;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using ::dart::dynamics::InverseKinematics;

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
  auto matchingNodes = mMetaSkeleton->getBodyNodes(
      problem.getEndEffectorBodyNode()->getName());
  if (matchingNodes.size() == 0)
    throw std::invalid_argument(
        "End-effector BodyNode not found in Planner's MetaSkeleton.");
  ::dart::dynamics::BodyNodePtr endEffectorBodyNode = matchingNodes.front();

  auto ik = InverseKinematics::create(endEffectorBodyNode);
  ik->setDofs(mMetaSkeleton->getDofs());

  // Create a sequential sampleable to provide seeds for IK solver
  auto space = std::const_pointer_cast<MetaSkeletonStateSpace>(
      mMetaSkeletonStateSpace);
  std::vector<constraint::ConstSampleablePtr> sampleables
      = {std::make_shared<FiniteSampleable>(space, problem.getStartState()),
         createSampleableBounds(
             mMetaSkeletonStateSpace, nullptr)}; // TODO: RNG -> Planner
  auto sampleable = std::make_shared<SequentialSampleable>(space, sampleables);

  // Convert TSR constraint into IK constraint
  InverseKinematicsSampleable ikSampleable(
      mMetaSkeletonStateSpace,
      mMetaSkeleton,
      std::const_pointer_cast<TSR>(problem.getGoalTSR()),
      sampleable,
      ik,
      problem.getMaxSamples());
  auto generator = ikSampleable.createSampleGenerator();

  // NOTE: Const-casting should be removed once InverseKinematicsSampleable is
  // changed to take const constraints!
  auto saver = MetaSkeletonStateSaver(mMetaSkeleton);
  DART_UNUSED(saver);

  auto sampleState = mMetaSkeletonStateSpace->createState();
  std::vector<statespace::StateSpace::State*> configurations;
  NominalConfigurationRanker ranker(
      mMetaSkeletonStateSpace, mMetaSkeleton, problem.getStartState());
  while (generator->canSample())
  {
    std::lock_guard<std::mutex> lock(skeleton->getMutex());
    bool sampled = generator->sample(sampleState);
    if (!sampled)
      continue;
    configurations.emplace_back(sampleState);
  }
  ranker.rankConfigurations(configurations);

  // Create ConfigurationToConfiguration Problem
  auto goalState = mMetaSkeletonStateSpace->createState();
  auto delegateProblem = ConfigurationToConfiguration(
      mMetaSkeletonStateSpace,
      problem.getStartState(),
      goalState,
      problem.getConstraint());

  // Set to start state
  for (auto configuration : configurations)
  {
    mMetaSkeletonStateSpace->setState(
        mMetaSkeleton.get(), problem.getStartState());
    mMetaSkeletonStateSpace->copyState(configuration, goalState);
    auto traj = mDelegate->plan(delegateProblem, result);
    if (traj)
      return traj;
  }

  return nullptr;
}

} // namespace dart
} // namespace planner
} // namespace aikido
