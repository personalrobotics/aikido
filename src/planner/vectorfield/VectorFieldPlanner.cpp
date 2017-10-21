#include <boost/format.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>
#include <aikido/trajectory/Spline.hpp>
#include "MoveHandStraightVectorField.hpp"
#include "VectorFieldPlannerExceptions.hpp"

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

  for (std::size_t i = 0; i < stateSpace->getMetaSkeleton()->getNumDofs(); ++i)
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

//==============================================================================

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
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    const aikido::constraint::TestablePtr _constraint,
    double _dt,
    const VectorFieldCallback& _vectorFiledCb,
    const VectorFieldStatusCallback& _statusCb)
{
  auto saver = MetaSkeletonStateSpaceSaver(_stateSpace);
  DART_UNUSED(saver);

  if (_stateSpace->getMetaSkeleton()->getPositionUpperLimits()
      == _stateSpace->getMetaSkeleton()->getPositionLowerLimits())
  {
    throw std::invalid_argument("State space volume zero");
  }
  if (_stateSpace->getMetaSkeleton()->getVelocityUpperLimits()
      == _stateSpace->getMetaSkeleton()->getVelocityLowerLimits())
  {
    throw std::invalid_argument("Velocity space volume zero");
  }

  const std::size_t numDof = _stateSpace->getDimension();

  std::vector<Knot> knots;
  VectorFieldPlannerStatus terminationStatus
      = VectorFieldPlannerStatus::CONTINUE;
  std::exception_ptr terminationError;

  ptrdiff_t cacheIndex = -1;
  ptrdiff_t index = 0;
  double t = 0;
  Eigen::VectorXd q = _stateSpace->getMetaSkeleton()->getPositions();

  auto startState = _stateSpace->createState();
  _stateSpace->convertPositionsToState(q, startState);
  Eigen::VectorXd qd(numDof);
  assert(static_cast<std::size_t>(q.size()) == numDof);

  do
  {
    // Evaluate the vector field.
    try
    {
      if (!_vectorFiledCb(_stateSpace, t, &qd))
      {
        throw VectorFieldTerminated("Failed evaluating VectorField.");
      }
    }
    catch (VectorFieldTerminated const& e)
    {
      terminationStatus = VectorFieldPlannerStatus::TERMINATE;
      terminationError = std::current_exception();
      break;
    }

    if (static_cast<std::size_t>(qd.size()) != numDof)
    {
      throw std::length_error(
          "Vector field returned an incorrect number of DOF velocities.");
    }

    // This should be computed from the DOF resolutions.
    const std::size_t numSteps = 1;

    // Compute the number of collision checks we need.
    for (std::size_t istep = 0; istep < numSteps; ++istep)
    {
      try
      {
        CheckDofLimits(_stateSpace, q, qd);
        _stateSpace->getMetaSkeleton()->setPositions(q);
        CheckCollision(_stateSpace, _constraint);
      }
      catch (VectorFieldTerminated const& e)
      {
        terminationStatus = VectorFieldPlannerStatus::TERMINATE;
        terminationError = std::current_exception();
        break;
      }

      // Insert the waypoint.
      assert(static_cast<std::size_t>(q.size()) == numDof);
      assert(static_cast<std::size_t>(qd.size()) == numDof);

      Knot knot;
      knot.mT = t;
      knot.mValues.resize(2, numDof);
      knot.mValues.row(0) = q;
      knot.mValues.row(1) = qd;
      knots.push_back(knot);

      // Take a step.
      auto currentState = _stateSpace->createState();
      _stateSpace->convertPositionsToState(q, currentState);
      auto deltaState = _stateSpace->createState();
      _stateSpace->convertPositionsToState(_dt * qd, deltaState);
      auto nextState = _stateSpace->createState();
      _stateSpace->compose(currentState, deltaState, nextState);
      _stateSpace->convertStateToPositions(nextState, q);

      t += _dt;
      index++;

      // Check if we should terminate.
      terminationStatus = _statusCb(_stateSpace, t);
      if (terminationStatus == VectorFieldPlannerStatus::CACHE_AND_CONTINUE
          || terminationStatus == VectorFieldPlannerStatus::CACHE_AND_TERMINATE)
      {
        cacheIndex = index;
      }
    }
  } while (terminationStatus != VectorFieldPlannerStatus::TERMINATE
           && terminationStatus
                  != VectorFieldPlannerStatus::CACHE_AND_TERMINATE);

  if (cacheIndex >= 0)
  {
    // Print the termination condition.
    if (terminationError)
    {
      try
      {
        std::rethrow_exception(terminationError);
      }
      catch (VectorFieldTerminated const& e)
      {
        dtwarn << "[VectorFieldPlanner::Plan] Terminated early: " << e.what()
               << '\n';
        return nullptr;
      }
    }

    return convertToSpline(knots, cacheIndex, _stateSpace);
  }
  // Re-throw whatever error caused planning to fail.
  else if (terminationError)
  {
    std::rethrow_exception(terminationError);
  }
  else
  {
    throw VectorFieldTerminated(
        "Planning was terminated by the StatusCallback.");
  }
  return nullptr;
}

//==============================================================================

std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    const dart::dynamics::BodyNodePtr _bn,
    const aikido::constraint::TestablePtr _constraint,
    const Eigen::Vector3d& _direction,
    double _distance,
    double _positionTolerance,
    double _angularTolerance,
    double _duration,
    double _timestep)
{
  if (_distance < 0.)
  {
    std::stringstream ss;
    ss << "Distance must be non-negative; got " << _distance << ".";
    throw std::runtime_error(ss.str());
  }

  if (_direction.norm() == 0.0)
  {
    throw std::runtime_error("Direction vector is a zero vector");
  }

  double linearVelocity = _distance / _duration;
  Eigen::Vector3d velocity = _direction.normalized() * linearVelocity;
  double linearGain = 1.0 / _timestep;
  double angularGain = 1.0 / _timestep;

  auto vectorfield = MoveHandStraightVectorField(
      _bn,
      velocity,
      0.0,
      _duration,
      _timestep,
      linearGain,
      _positionTolerance,
      angularGain,
      _angularTolerance);

  return planPathByVectorField(
      _stateSpace, _constraint, _timestep, vectorfield, vectorfield);
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
