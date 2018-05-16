#include "aikido/planner/vectorfield/VectorFieldConfigurationToEndEffectorOffsetPlanner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
VectorFieldConfigurationToEndEffectorOffsetPlanner::
    VectorFieldConfigurationToEndEffectorOffsetPlanner(
        statespace::ConstStateSpacePtr stateSpace,
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
trajectory::TrajectoryPtr VectorFieldConfigurationToEndEffectorOffsetPlanner::plan(
    const SolvableProblem& problem, Result* result)
{
  // TODO (sniyaz): Check equality between state space of this planner and given
  // problem.

  using aikido::planner::vectorfield::planToEndEffectorOffset;

  // Just call the core VFP function.
  // TODO: How should start state be handled?
  return planToEndEffectorOffset(
    mStateSpace,
    mMetaskeleton,
    problem.getEndEffectorBodyNode(),
    problem.getConstraint(),
    problem.getDirection(),
    // TODO: Need to handle case when negative.
    problem.getDistance() - mDistanceTolerance,
    problem.getDistance() + mDistanceTolerance,
    mPositionTolerance,
    mAngularTolerance,
    mInitialStepSize,
    mJointLimitTolerance,
    mConstraintCheckResolution,
    mTimelimit,
    result
  );
}

} // namespace planner
} // namespace aikido
