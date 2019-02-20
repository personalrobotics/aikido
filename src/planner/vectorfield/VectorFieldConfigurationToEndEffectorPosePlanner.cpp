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
        double poseTolerance,
        double conversionRatioInGeodesicDistance,
        double initialStepSize,
        double jointLimitTolerance,
        double constraintCheckResolution,
        std::chrono::duration<double> timelimit)
  : ConfigurationToEndEffectorPosePlanner(
        std::move(stateSpace), std::move(metaSkeleton))
  , mPoseTolerance(poseTolerance)
  , mConversionRatioInGeodesicDistance(conversionRatioInGeodesicDistance)
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
  // TODO (sniyaz): Check equality between state space of this planner and given
  // problem.

  using aikido::planner::vectorfield::planToEndEffectorPose;
  using aikido::statespace::dart::MetaSkeletonStateSpace;

  auto pose = problem.getGoalPose();

  // Just call the core VFP function.
  return planToEndEffectorPose(
      getMetaSkeletonStateSpace(),
      mMetaSkeleton,
      problem.getEndEffectorBodyNode(),
      problem.getConstraint(),
      pose,
      mPoseTolerance,
      mConversionRatioInGeodesicDistance,
      mInitialStepSize,
      mJointLimitTolerance,
      mConstraintCheckResolution,
      mTimelimit,
      result);
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
