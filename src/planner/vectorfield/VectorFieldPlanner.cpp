#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorPoseVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldIntegrator.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>
#include <aikido/trajectory/Spline.hpp>

using aikido::planner::vectorfield::VectorFieldIntegrator;
using aikido::planner::vectorfield::MoveEndEffectorOffsetVectorField;
using aikido::planner::vectorfield::MoveEndEffectorPoseVectorField;
using aikido::statespace::dart::MetaSkeletonStateSpaceSaver;

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Vector3d& direction,
    double minDistance,
    double maxDistance,
    double positionTolerance,
    double angularTolerance,
    double linearVelocityGain,
    double initialStepSize,
    double jointLimitTolerance,
    double constraintCheckResolution,
    std::chrono::duration<double> timelimit,
    planner::PlanningResult* planningResult)
{
  if (minDistance < 0.)
  {
    std::stringstream ss;
    ss << "Distance must be non-negative; got " << minDistance << ".";
    throw std::runtime_error(ss.str());
  }

  if (maxDistance < minDistance)
  {
    throw std::runtime_error("Max distance is less than distance.");
  }

  if (direction.norm() == 0.0)
  {
    throw std::runtime_error("Direction vector is a zero vector");
  }

  // Save the current state of the space
  auto saver = MetaSkeletonStateSpaceSaver(stateSpace);
  DART_UNUSED(saver);

  auto vectorfield = std::make_shared<MoveEndEffectorOffsetVectorField>(
      stateSpace,
      bn,
      direction,
      minDistance,
      maxDistance,
      positionTolerance,
      angularTolerance,
      linearVelocityGain,
      initialStepSize,
      jointLimitTolerance);

  auto planner
      = std::make_shared<VectorFieldIntegrator>(vectorfield, constraint);

  auto startState = stateSpace->getScopedStateFromMetaSkeleton();
  return planner->followVectorField(
      startState,
      timelimit,
      initialStepSize,
      constraintCheckResolution,
      planningResult);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorPose(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Isometry3d& goalPose,
    double poseErrorTolerance,
    double conversionRatioInGeodesicDistance,
    double linearVelocityGain,
    double angularvelocityGain,
    double initialStepSize,
    double jointLimitTolerance,
    double constraintCheckResolution,
    std::chrono::duration<double> timelimit,
    planner::PlanningResult* planningResult)
{
  // Save the current state of the space
  auto saver = MetaSkeletonStateSpaceSaver(stateSpace);
  DART_UNUSED(saver);

  auto vectorfield = std::make_shared<MoveEndEffectorPoseVectorField>(
      stateSpace,
      bn,
      goalPose,
      poseErrorTolerance,
      conversionRatioInGeodesicDistance,
      linearVelocityGain,
      angularvelocityGain,
      initialStepSize,
      jointLimitTolerance);

  auto planner
      = std::make_shared<VectorFieldIntegrator>(vectorfield, constraint);

  auto startState = stateSpace->getScopedStateFromMetaSkeleton();
  return planner->followVectorField(
      startState,
      timelimit,
      initialStepSize,
      constraintCheckResolution,
      planningResult);
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
