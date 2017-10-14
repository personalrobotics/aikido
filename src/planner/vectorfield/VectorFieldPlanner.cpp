#include <boost/format.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>
#include <aikido/trajectory/Spline.hpp>
#include "MoveHandStraightVectorField.hpp"
#include "VectorFieldPlannerExceptions.hpp"

using dart::common::make_unique;
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

  struct Knot
  {
    double t;
    Eigen::Matrix<double, 2, Eigen::Dynamic> values;
  };

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

  size_t const num_dof = stateSpace->getDimension();

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

    if (static_cast<size_t>(qd.size()) != num_dof)
    {
      throw std::length_error(
          "Vector field returned an incorrect number of DOF velocities.");
    }

    // TODO: This should be computed from the DOF resolutions.
    size_t const num_steps = 1;

    // Compute the number of collision checks we need.
    for (size_t istep = 0; istep < num_steps; ++istep)
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
        cache_index = index;

        // Take a step.
        q += dt * qd;
        t += dt;
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
      }
    }

    // Construct the output spline.
    Eigen::VectorXd times(cache_index);
    std::transform(
        knots.begin(),
        knots.begin() + cache_index,
        times.data(),
        [](Knot const& knot) { return knot.t; });

    auto _outputTrajectory
        = make_unique<aikido::trajectory::Spline>(stateSpace);

    using CubicSplineProblem = aikido::common::
        SplineProblem<double, int, 4, Eigen::Dynamic, Eigen::Dynamic>;

    CubicSplineProblem problem(times, 4, num_dof);
    for (int iknot = 0; iknot < cache_index; ++iknot)
    {
      problem.addConstantConstraint(iknot, 0, knots[iknot].values.row(0));

      if (iknot != 0 && iknot != cache_index - 1)
      {
        problem.addContinuityConstraint(iknot, 1);
        problem.addContinuityConstraint(iknot, 2);
      }
    }

    problem.addConstantConstraint(0, 1, knots.front().values.row(1));
    problem.addConstantConstraint(
        cache_index - 1, 1, knots[cache_index - 1].values.row(1));

    const auto spline = problem.fit();

    // convert spline to outputTrajectory
    for (size_t i = 0; i < spline.getNumKnots() - 1; i++)
    {
      auto coefficients = spline.getCoefficients()[i];
      double timeStep = spline.getTimes()[i];
      double duration = spline.getTimes()[i + 1] - spline.getTimes()[i];
      auto currentVec = spline.evaluate(timeStep);
      auto currentState = stateSpace->createState();
      stateSpace->convertPositionsToState(currentVec, currentState);
      _outputTrajectory->addSegment(coefficients, duration, currentState);
    }

    return _outputTrajectory;
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
}

std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const dart::dynamics::BodyNodePtr bn,
    const aikido::constraint::TestablePtr constraint,
    const Eigen::Vector3d& direction,
    double distance,
    double position_tolerance,
    double angular_tolerance,
    double integration_interval)
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

  double dt = 0.01;
  double linear_gain = 10.0;
  double angular_gain = 10.0;

  double linear_velocity = distance / integration_interval;

  auto vectorfield = MoveHandStraightVectorField(
      bn,
      direction.normalized() * linear_velocity,
      0.0,
      integration_interval,
      dt,
      linear_gain,
      position_tolerance,
      angular_gain,
      angular_tolerance);

  return planPathByVectorField(
      stateSpace, constraint, dt, vectorfield, vectorfield);
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
