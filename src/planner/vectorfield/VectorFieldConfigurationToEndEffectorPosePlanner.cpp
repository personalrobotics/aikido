#include "aikido/planner/vectorfield/VectorFieldConfigurationToEndEffectorPosePlanner.hpp"

#include "aikido/planner/vectorfield/VectorFieldPlanner.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
VectorFieldConfigurationToEndEffectorPosePlanner::
    VectorFieldConfigurationToEndEffectorPosePlanner(
        statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
        ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
        double goalTolerance,
        double anglePoseRatio,
        double positionTolerance,
        double angularTolerance,
        double initialStepSize,
        double jointLimitTolerance,
        double constraintCheckResolution,
        std::chrono::duration<double> timelimit)
  : ConfigurationToEndEffectorPosePlanner(
      std::move(stateSpace), std::move(metaSkeleton))
  , mGoalTolerance(goalTolerance)
  , mAnglePoseRatio(anglePoseRatio)
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
VectorFieldConfigurationToEndEffectorPosePlanner::plan(
    const SolvableProblem& problem, Result* result)
{
  // TODO (egordon): Check equality between state space of this planner and
  // given problem.

  using aikido::planner::vectorfield::planToEndEffectorPose;

  // Just call the core VFP function.
  // NOTE: This both locks the metaskeleton while planning and restores its
  // state after planning.
  return planToEndEffectorPose(
      getMetaSkeletonStateSpace(),
      *problem.getStartState(),
      mMetaSkeleton,
      problem.getEndEffectorBodyNode(),
      problem.getConstraint(),
      problem.getGoalPose(),
      mGoalTolerance,
      mAnglePoseRatio,
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
