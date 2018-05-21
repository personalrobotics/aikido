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

  auto metaskeletonStateSpace = getMetaSkeletonStateSpace();

  if (problem.getDistance() - mDistanceTolerance < 0.)
  {
    std::stringstream ss;
    ss << "Distance must be non-negative; min distance to move is "
       << problem.getDistance() - mDistanceTolerance << ".";
    throw std::runtime_error(ss.str());
  }

  // Just call the core VFP function.
  // TODO: How should start state be handled?
  return planToEndEffectorOffset(
      metaskeletonStateSpace,
      mMetaskeleton,
      problem.getEndEffectorBodyNode(),
      problem.getConstraint(),
      problem.getDirection(),
      problem.getDistance() - mDistanceTolerance,
      problem.getDistance() + mDistanceTolerance,
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
