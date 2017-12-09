#include <boost/numeric/odeint.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorPoseVectorField.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorAlongWorkspacePathVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>
#include <aikido/trajectory/Spline.hpp>
#include "detail/BodyNodePoseVectorFieldConstraint.hpp"
#include "detail/VectorFieldIntegrator.hpp"
#include "detail/VectorFieldPlannerExceptions.hpp"

using aikido::planner::vectorfield::detail::VectorFieldIntegrator;
using aikido::planner::vectorfield::MoveEndEffectorOffsetVectorField;
using aikido::planner::vectorfield::MoveEndEffectorPoseVectorField;
using aikido::planner::vectorfield::MoveEndEffectorAlongWorkspacePathVectorField;
using aikido::statespace::dart::MetaSkeletonStateSpaceSaver;

namespace aikido {
namespace planner {
namespace vectorfield {

constexpr double integrationTimeInterval = 10.0;

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> followVectorField(
    const aikido::planner::vectorfield::VectorField* vectorField,
    const aikido::statespace::StateSpace::State* startState,
    const aikido::constraint::Testable* constraint,
    std::chrono::duration<double> timelimit,
    double initialStepSize,
    double checkConstraintResolution,
    planner::PlanningResult* planningResult)
{
  using namespace std::placeholders;
  using errorStepper = boost::numeric::odeint::
      runge_kutta_dopri5<Eigen::VectorXd,
                         double,
                         Eigen::VectorXd,
                         double,
                         boost::numeric::odeint::vector_space_algebra>;

  std::size_t dimension = vectorField->getStateSpace()->getDimension();
  auto integrator = std::make_shared<detail::VectorFieldIntegrator>(
      vectorField, constraint, timelimit.count(), checkConstraintResolution);

  integrator->start();

  try
  {
    Eigen::VectorXd initialQ(dimension);
    vectorField->getStateSpace()->logMap(startState, initialQ);

    // Integrate the vector field to get a configuration space path.
    boost::numeric::odeint::integrate_adaptive(
        errorStepper(),
        std::bind(&detail::VectorFieldIntegrator::step, integrator, _1, _2, _3),
        initialQ,
        0.,
        integrationTimeInterval,
        initialStepSize,
        std::bind(&detail::VectorFieldIntegrator::check, integrator, _1, _2));
  }
  catch (const detail::VectorFieldTerminated& e)
  {
    // dtwarn << e.what() << std::endl;
    if (planningResult)
    {
      planningResult->message = e.what();
    }
  }
  catch (const detail::VectorFieldError& e)
  {
    dtwarn << e.what() << std::endl;
    if (planningResult)
    {
      planningResult->message = e.what();
    }
    return nullptr;
  }

  if (integrator->getCacheIndex() < 0)
  {
    if (planningResult)
    {
      planningResult->message = "No waypoint cached.";
    }
    return nullptr;
  }

  std::vector<detail::Knot> newKnots(
      integrator->getKnots().begin(),
      integrator->getKnots().begin() + integrator->getCacheIndex());
  return detail::convertToSpline(newKnots, vectorField->getStateSpace());
}

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
      initialStepSize,
      jointLimitTolerance);

  auto combinedConstraint
      = std::make_shared<detail::BodyNodePoseVectorFieldConstraint>(
          stateSpace, constraint);

  auto startState = stateSpace->getScopedStateFromMetaSkeleton();
  return followVectorField(
      vectorfield.get(),
      startState,
      combinedConstraint.get(),
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

  auto combinedConstraint
      = std::make_shared<detail::BodyNodePoseVectorFieldConstraint>(
          stateSpace, constraint);

  auto startState = stateSpace->getScopedStateFromMetaSkeleton();
  return followVectorField(
      vectorfield.get(),
      startState,
      combinedConstraint.get(),
      timelimit,
      initialStepSize,
      constraintCheckResolution,
      planningResult);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> planWorkspacePath(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const aikido::trajectory::InterpolatedPtr workspacePath,
    double positionTolerance,
    double angularTolerance,
    double tStep,
    const Eigen::Vector6d& kpFF,
    const Eigen::Vector6d& kpE,
    double initialStepSize,
    double jointLimitTolerance,
    double constraintCheckResolution,
    std::chrono::duration<double> timelimit,
    planner::PlanningResult* planningResult)
{
  auto vectorfield
      = std::make_shared<MoveEndEffectorAlongWorkspacePathVectorField>(
          stateSpace,
          bn,
          workspacePath,
          positionTolerance,
          angularTolerance,
          tStep,
          initialStepSize,
          jointLimitTolerance,
          kpFF,
          kpE);

  auto combinedConstraint
      = std::make_shared<detail::BodyNodePoseVectorFieldConstraint>(
          stateSpace, constraint);

  auto startState = stateSpace->getScopedStateFromMetaSkeleton();
  return followVectorField(
      vectorfield.get(),
      startState,
      combinedConstraint.get(),
      timelimit,
      initialStepSize,
      constraintCheckResolution,
      planningResult);
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
