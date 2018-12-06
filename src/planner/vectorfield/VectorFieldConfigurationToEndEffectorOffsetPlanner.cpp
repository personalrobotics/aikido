#include "aikido/planner/vectorfield/VectorFieldConfigurationToEndEffectorOffsetPlanner.hpp"

#include "aikido/planner/vectorfield/VectorFieldPlanner.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
VectorFieldConfigurationToEndEffectorOffsetPlanner::
    VectorFieldConfigurationToEndEffectorOffsetPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
        ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
        double distanceTolerance,
        double positionTolerance,
        double angularTolerance,
        double initialStepSize,
        double jointLimitTolerance,
        double constraintCheckResolution,
        std::chrono::duration<double> timelimit)
  : ConfigurationToEndEffectorOffsetPlanner(
        std::move(stateSpace), std::move(metaSkeleton))
  , mDistanceTolerance(distanceTolerance)
  , mPositionTolerance(positionTolerance)
  , mAngularTolerance(angularTolerance)
  , mInitialStepSize(initialStepSize)
  , mJointLimitTolerance(jointLimitTolerance)
  , mConstraintCheckResolution(constraintCheckResolution)
  , mTimelimit(timelimit)
{
  if (endEffectorBodyNode)
  {
    std::cout << "Set endeffectorBodyNode" << std::endl;
    setEndEffectorBodyNode(endEffectorBodyNode);
  }
  // Do nothing here.
}

//==============================================================================
trajectory::TrajectoryPtr
VectorFieldConfigurationToEndEffectorOffsetPlanner::plan(
    const SolvableProblem& problem, Result* result)
{
  if (!mEndEffectorBodyNode)
    throw std::runtime_error(
        "VectorFieldConfigurationToEndEffectorOffsetPlanner needs mEndEffectorBodyNode");

  // TODO (sniyaz): Check equality between state space of this planner and given
  // problem.

  using aikido::planner::vectorfield::planToEndEffectorOffset;
  using aikido::statespace::dart::MetaSkeletonStateSpace;

  // Handle the fact that distance can be negative.
  double distance = problem.getDistance();
  auto optionalDirection = problem.getDirection();
  Eigen::Vector3d direction;
  if (!optionalDirection)
    direction = getEndEffectorDirection();
  else
  {
    std::cout << "Problem direction is used for VectorFieldConfigurationToEndEffectorOffsetPlanner" << std::endl;
    direction = optionalDirection.get().normalized();
  }
  if (distance < 0)
  {
    distance = -1.0 * distance;
    direction = -1.0 * direction;
  }

  double minDistance = distance - mDistanceTolerance;
  double maxDistance = distance + mDistanceTolerance;

  if (minDistance < 0.)
  {
    std::stringstream ss;
    ss << "Distance must be non-negative; min distance to move is "
       << problem.getDistance() - mDistanceTolerance << ".";
    throw std::runtime_error(ss.str());
  }

  // Just call the core VFP function.
  return planToEndEffectorOffset(
      getMetaSkeletonStateSpace(),
      *problem.getStartState(),
      mMetaSkeleton,
      mEndEffectorBodyNode,
      problem.getConstraint(),
      direction,
      minDistance,
      maxDistance,
      mPositionTolerance,
      mAngularTolerance,
      mInitialStepSize,
      mJointLimitTolerance,
      mConstraintCheckResolution,
      mTimelimit,
      result);
}

//==============================================================================
PlannerPtr VectorFieldConfigurationToEndEffectorOffsetPlanner::clone(
    common::RNG* rng) const
{
  return std::make_shared<VectorFieldConfigurationToEndEffectorOffsetPlanner>(
      mMetaSkeletonStateSpace,
      mMetaSkeleton,
      mEndEffectorBodyNode,
      mDistanceTolerance,
      mPositionTolerance,
      mAngularTolerance,
      mInitialStepSize,
      mJointLimitTolerance,
      mConstraintCheckResolution,
      mTimelimit
   );
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
