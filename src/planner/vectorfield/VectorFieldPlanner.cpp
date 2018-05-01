#include <boost/numeric/odeint.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/constraint/dart/JointStateSpaceHelpers.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorPoseVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/trajectory/Spline.hpp>
#include "detail/VectorFieldIntegrator.hpp"
#include "detail/VectorFieldPlannerExceptions.hpp"

using aikido::planner::vectorfield::detail::VectorFieldIntegrator;
using aikido::planner::vectorfield::MoveEndEffectorOffsetVectorField;
using aikido::planner::vectorfield::MoveEndEffectorPoseVectorField;
using aikido::statespace::dart::MetaSkeletonStateSaver;

namespace aikido {
namespace planner {
namespace vectorfield {

constexpr double integrationTimeInterval = 10.0;

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> followVectorField(
    const aikido::planner::vectorfield::VectorField& vectorField,
    const aikido::statespace::StateSpace::State& startState,
    const aikido::constraint::Testable& constraint,
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

  std::size_t dimension = vectorField.getStateSpace()->getDimension();
  auto integrator = std::make_shared<detail::VectorFieldIntegrator>(
      &vectorField, &constraint, timelimit.count(), checkConstraintResolution);

  integrator->start();

  try
  {
    Eigen::VectorXd initialQ(dimension);
    vectorField.getStateSpace()->logMap(&startState, initialQ);
    // The current implementation works only in real vector spaces.

    // Integrate the vector field to get a configuration space path.
    boost::numeric::odeint::integrate_adaptive(
        errorStepper(),
        std::bind(
            &detail::VectorFieldIntegrator::step,
            integrator,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3),
        initialQ,
        0.,
        integrationTimeInterval,
        initialStepSize,
        std::bind(
            &detail::VectorFieldIntegrator::check,
            integrator,
            std::placeholders::_1,
            std::placeholders::_2));
  }
  // VectorFieldTerminated is an exception that is raised internally to
  // terminate
  // integration, which does not indicate that an error has occurred.
  catch (const detail::VectorFieldTerminated& e)
  {
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

  if (integrator->getCacheIndex() <= 1)
  {
    // no enough waypoints cached to make a trajectory output.
    if (planningResult)
    {
      planningResult->message = "No segment cached.";
    }
    return nullptr;
  }

  std::vector<detail::Knot> newKnots(
      integrator->getKnots().begin(),
      integrator->getKnots().begin() + integrator->getCacheIndex());
  auto outputTrajectory
      = detail::convertToSpline(newKnots, vectorField.getStateSpace());

  // evaluate constraint satisfaction on last piece of trajectory
  double lastEvaluationTime = integrator->getLastEvaluationTime();
  if (outputTrajectory->getEndTime() > lastEvaluationTime)
  {
    if (!vectorField.evaluateTrajectory(
            *outputTrajectory,
            &constraint,
            checkConstraintResolution,
            lastEvaluationTime,
            true))
    {
      planningResult->message = "Constraint violated.";
      return nullptr;
    }
  }
  return outputTrajectory;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    dart::dynamics::MetaSkeletonPtr metaskeleton,
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
  // ensure that no two planners run at the same time
  if (metaskeleton->getNumBodyNodes() <= 0)
  {
      throw std::runtime_error("MetaSkeleton doesn't have any body nodes.");
  }
  auto robot = metaskeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());

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

  // TODO: Check compatibility between MetaSkeleton and MetaSkeletonStateSpace

  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(
      metaskeleton, MetaSkeletonStateSaver::Options::POSITIONS);
  DART_UNUSED(saver);

  auto vectorfield
      = dart::common::make_aligned_shared<MoveEndEffectorOffsetVectorField>(
          stateSpace,
          metaskeleton,
          bn,
          direction,
          minDistance,
          maxDistance,
          positionTolerance,
          angularTolerance,
          initialStepSize,
          jointLimitTolerance);

  auto compoundConstraint
      = std::make_shared<constraint::TestableIntersection>(stateSpace);
  compoundConstraint->addConstraint(constraint);
  compoundConstraint->addConstraint(
      constraint::dart::createTestableBounds(stateSpace));

  auto startState
      = stateSpace->getScopedStateFromMetaSkeleton(metaskeleton.get());
  return followVectorField(
      *vectorfield,
      *startState,
      *compoundConstraint,
      timelimit,
      initialStepSize,
      constraintCheckResolution,
      planningResult);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorPose(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    dart::dynamics::MetaSkeletonPtr metaskeleton,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Isometry3d& goalPose,
    double poseErrorTolerance,
    double conversionRatioInGeodesicDistance,
    double initialStepSize,
    double jointLimitTolerance,
    double constraintCheckResolution,
    std::chrono::duration<double> timelimit,
    planner::PlanningResult* planningResult)
{
  // TODO: Check compatibility between MetaSkeleton and MetaSkeletonStateSpace

  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(
      metaskeleton, MetaSkeletonStateSaver::Options::POSITIONS);
  DART_UNUSED(saver);

  auto vectorfield
      = dart::common::make_aligned_shared<MoveEndEffectorPoseVectorField>(
          stateSpace,
          metaskeleton,
          bn,
          goalPose,
          poseErrorTolerance,
          conversionRatioInGeodesicDistance,
          initialStepSize,
          jointLimitTolerance);

  auto compoundConstraint
      = std::make_shared<aikido::constraint::TestableIntersection>(stateSpace);
  compoundConstraint->addConstraint(constraint);
  compoundConstraint->addConstraint(
      constraint::dart::createTestableBounds(stateSpace));

  auto startState
      = stateSpace->getScopedStateFromMetaSkeleton(metaskeleton.get());
  return followVectorField(
      *vectorfield,
      *startState,
      *compoundConstraint,
      timelimit,
      initialStepSize,
      constraintCheckResolution,
      planningResult);
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
