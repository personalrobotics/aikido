#include "aikido/planner/vectorfield/VectorFieldConfigurationToEndEffectorOffsetPlanner.hpp"

#include "aikido/planner/vectorfield/VectorFieldPlanner.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
VectorFieldConfigurationToEndEffectorOffsetPlanner::
    VectorFieldConfigurationToEndEffectorOffsetPlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        dart::dynamics::MetaSkeletonPtr metaskeleton,
        double distanceTolerance,
        double positionTolerance,
        double angularTolerance,
        double initialStepSize,
        double jointLimitTolerance,
        double constraintCheckResolution,
        std::chrono::duration<double> timelimit)
  : ConfigurationToEndEffectorOffsetPlanner(std::move(stateSpace))
  , mMetaskeleton(std::move(metaskeleton))
  , mDistanceTolerance(distanceTolerance)
  , mPositionTolerance(positionTolerance)
  , mAngularTolerance(angularTolerance)
  , mInitialStepSize(initialStepSize)
  , mJointLimitTolerance(jointLimitTolerance)
  , mConstraintCheckResolution(constraintCheckResolution)
  , mTimelimit(timelimit)
{
  // Do nothing here.
}

//==============================================================================
trajectory::TrajectoryPtr
VectorFieldConfigurationToEndEffectorOffsetPlanner::plan(
    const SolvableProblem& problem, Result* result)
{
  // TODO (sniyaz): Check equality between state space of this planner and given
  // problem.

  using aikido::planner::vectorfield::planToEndEffectorOffset;
  using aikido::statespace::dart::MetaSkeletonStateSpace;

  // Handle the fact that distance can be negative.
  double distance = problem.getDistance();
  Eigen::Vector3d direction = problem.getDirection();

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
  // TODO: How should start state be handled?
  return planToEndEffectorOffset(
      *problem.getStartState(),
      getMetaSkeletonStateSpace(),
      mMetaskeleton,
      problem.getEndEffectorBodyNode(),
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

} // namespace vectorfield
} // namespace planner
} // namespace aikido
