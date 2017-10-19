#include <boost/format.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>
#include <aikido/trajectory/Spline.hpp>
#include "MoveHandStraightVectorField.hpp"
#include "VectorFieldPlannerExceptions.hpp"
#include "VectorFieldUtil.hpp"

using aikido::statespace::dart::MetaSkeletonStateSpaceSaver;

namespace aikido {
namespace planner {
namespace vectorfield {

static void CheckDofLimits(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    Eigen::VectorXd const& q,
    Eigen::VectorXd const& qd)
{
  using boost::format;
  using boost::str;
  using dart::dynamics::DegreeOfFreedom;

  for (size_t i = 0; i < stateSpace->getMetaSkeleton()->getNumDofs(); ++i)
  {
    DegreeOfFreedom* const dof = stateSpace->getMetaSkeleton()->getDof(i);

    if (q[i] < dof->getPositionLowerLimit())
    {
      throw DofLimitError(
          dof,
          str(format("DOF '%s' exceeds lower position limit: %f < %f")
              % dof->getName()
              % q[i]
              % dof->getPositionLowerLimit()));
    }
    else if (q[i] > dof->getPositionUpperLimit())
    {
      throw DofLimitError(
          dof,
          str(format("DOF '%s' exceeds upper position limit: %f > %f")
              % dof->getName()
              % q[i]
              % dof->getPositionUpperLimit()));
    }
    else if (qd[i] < dof->getVelocityLowerLimit())
    {
      throw DofLimitError(
          dof,
          str(format("DOF '%s' exceeds lower velocity limit: %f < %f")
              % dof->getName()
              % qd[i]
              % dof->getVelocityLowerLimit()));
    }
    else if (qd[i] > dof->getVelocityUpperLimit())
    {
      throw DofLimitError(
          dof,
          str(format("DOF '%s' exceeds upper velocity limit: %f > %f")
              % dof->getName()
              % qd[i]
              % dof->getVelocityUpperLimit()));
    }
  }
}

static void CheckCollision(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const aikido::constraint::TestablePtr constraint)
{
  // Get current position
  auto state = stateSpace->getScopedStateFromMetaSkeleton();
  // Throw a termination if in collision
  if (!constraint->isSatisfied(state))
  {
    throw VectorFieldTerminated("state in collision");
  }
}

std::unique_ptr<aikido::trajectory::Spline> planPathByVectorField(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const aikido::constraint::TestablePtr constraint,
    double dt,
    const VectorFieldCallback& vectorFiledCb,
    const VectorFieldStatusCallback& statusCb)
{
  using boost::format;
  using boost::str;
  using dart::dynamics::DegreeOfFreedom;
  using dart::dynamics::DegreeOfFreedomPtr;
  using aikido::trajectory::Spline;


  auto saver = MetaSkeletonStateSpaceSaver(stateSpace);
  DART_UNUSED(saver);

  if (stateSpace->getMetaSkeleton()->getPositionUpperLimits()
      == stateSpace->getMetaSkeleton()->getPositionLowerLimits())
  {
    throw std::invalid_argument("State space volume zero");
  }
  if (stateSpace->getMetaSkeleton()->getVelocityUpperLimits()
      == stateSpace->getMetaSkeleton()->getVelocityLowerLimits())
  {
    throw std::invalid_argument("Velocity space volume zero");
  }

  std::size_t const num_dof = stateSpace->getDimension();

  std::vector<Knot> knots;
  VectorFieldPlannerStatus::Enum termination_status
      = VectorFieldPlannerStatus::CONTINUE;
  std::exception_ptr termination_error;

  ptrdiff_t cache_index = -1;
  ptrdiff_t index = 0;
  double t = 0;
  Eigen::VectorXd q = stateSpace->getMetaSkeleton()->getPositions();

  auto startState = stateSpace->createState();
  stateSpace->convertPositionsToState(q, startState);
  Eigen::VectorXd qd(num_dof);
  assert(static_cast<size_t>(q.size()) == num_dof);

  do
  {
    // Evaluate the vector field.
    try
    {
      if (!vectorFiledCb(stateSpace, t, &qd))
      {
        throw VectorFieldTerminated("Failed evaluating VectorField.");
      }
    }
    catch (VectorFieldTerminated const& e)
    {
      termination_status = VectorFieldPlannerStatus::TERMINATE;
      termination_error = std::current_exception();
      break;
    }

    if (static_cast<std::size_t>(qd.size()) != num_dof)
    {
      throw std::length_error(
          "Vector field returned an incorrect number of DOF velocities.");
    }

    // TODO: This should be computed from the DOF resolutions.
    std::size_t const num_steps = 1;

    // Compute the number of collision checks we need.
    for (std::size_t istep = 0; istep < num_steps; ++istep)
    {
      try
      {
        CheckDofLimits(stateSpace, q, qd);
        stateSpace->getMetaSkeleton()->setPositions(q);
        CheckCollision(stateSpace, constraint);
      }
      catch (VectorFieldTerminated const& e)
      {
        termination_status = VectorFieldPlannerStatus::TERMINATE;
        termination_error = std::current_exception();
        break;
      }

      // Insert the waypoint.
      assert(static_cast<size_t>(q.size()) == num_dof);
      assert(static_cast<size_t>(qd.size()) == num_dof);

      Knot knot;
      knot.t = t;
      knot.values.resize(2, num_dof);
      knot.values.row(0) = q;
      knot.values.row(1) = qd;
      knots.push_back(knot);

      index++;

      // Check if we should terminate.
      termination_status = statusCb(stateSpace, t);
      if (termination_status == VectorFieldPlannerStatus::CACHE_AND_CONTINUE
          || termination_status
                 == VectorFieldPlannerStatus::CACHE_AND_TERMINATE)
      {

        // Take a step.
        q += dt * qd;
        t += dt;
        cache_index = index;
      }
    }
  } while (termination_status != VectorFieldPlannerStatus::TERMINATE
           && termination_status
                  != VectorFieldPlannerStatus::CACHE_AND_TERMINATE);

  if (cache_index >= 0)
  {
    // Print the termination condition.
    if (termination_error)
    {
      try
      {
        std::rethrow_exception(termination_error);
      }
      catch (VectorFieldTerminated const& e)
      {
        dtwarn << "[VectorFieldPlanner::Plan] Terminated early: " << e.what()
               << '\n';
        return nullptr;
      }
    }

    return convertToSpline(knots, cache_index, stateSpace);
  }
  // Re-throw whatever error caused planning to fail.
  else if (termination_error)
  {
    std::rethrow_exception(termination_error);
  }
  else
  {
    throw VectorFieldTerminated(
        "Planning was terminated by the StatusCallback.");
  }
  return nullptr;
}

std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const dart::dynamics::BodyNodePtr bn,
    const aikido::constraint::TestablePtr constraint,
    const Eigen::Vector3d& direction,
    double distance,
    double positionTolerance,
    double angularTolerance,
    double duration,
    double timestep,
    double linearGain,
    double angularGain)
{
  if (distance < 0.)
  {
    std::stringstream ss;
    ss << "Distance must be non-negative; got " << distance << ".";
    throw std::runtime_error(ss.str());
  }

  if (direction.norm() == 0.0)
  {
    throw std::runtime_error("Direction vector is a zero vector");
  }

  double linear_velocity = distance / duration;
  Eigen::Vector3d velocity = direction.normalized() * linear_velocity;

  auto vectorfield = MoveHandStraightVectorField(
      bn,
      velocity,
      0.0,
      duration,
      timestep,
      linearGain,
      positionTolerance,
      angularGain,
      angularTolerance);

  return planPathByVectorField(
      stateSpace, constraint, timestep, vectorfield, vectorfield);
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
