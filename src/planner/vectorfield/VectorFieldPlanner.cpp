#include <exception>
#include <string>
#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/trajectory/Spline.hpp>

using aikido::statespace::dart::MetaSkeletonStateSaver;

namespace aikido {
namespace planner {
namespace vectorfield {

static void checkDofLimits(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    Eigen::VectorXd const& q,
    Eigen::VectorXd const& qd)
{
  const auto properties = stateSpace->getProperties();
  const auto dofNames = properties.getDofNames();
  const auto positionLowerLimits = properties.getPositionLowerLimits();
  const auto positionUpperLimits = properties.getPositionUpperLimits();
  const auto velocityLowerLimits = properties.getVelocityLowerLimits();
  const auto velocityUpperLimits = properties.getVelocityUpperLimits();

  std::stringstream ss;

  for (std::size_t i = 0; i < properties.getNumDofs(); ++i)
  {
    if (q[i] < positionLowerLimits[i])
    {
      ss << "DOF " << dofNames[i] << " exceeds lower position limit: ";
      ss << q[i] << " < " << positionLowerLimits[i];
      throw DofLimitError(dofNames[i], ss.str());
    }
    else if (q[i] > positionUpperLimits[i])
    {
      ss << "DOF " << dofNames[i] << " exceeds upper position limit: ";
      ss << q[i] << " > " << positionUpperLimits[i];
      throw DofLimitError(dofNames[i], ss.str());
    }
    else if (qd[i] < velocityLowerLimits[i])
    {
      ss << "DOF " << dofNames[i] << " exceeds lower velocity limit: ";
      ss << qd[i] << " < " << velocityLowerLimits[i];
      throw DofLimitError(dofNames[i], ss.str());
    }
    else if (qd[i] > velocityUpperLimits[i])
    {
      ss << "DOF " << dofNames[i] << " exceeds upper velocity limit: ";
      ss << qd[i] << " > " << velocityUpperLimits[i];
      throw DofLimitError(dofNames[i], ss.str());
    }
  }
}

//==============================================================================
static void checkCollision(
    const aikido::statespace::dart::MetaSkeletonStateSpace::State* state,
    const aikido::constraint::TestablePtr& constraint)
{
  // Throw a termination if in collision
  if (!constraint->isSatisfied(state))
  {
    throw VectorFieldTerminated("state in collision");
  }
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> planPathByVectorField(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& _stateSpace,
    dart::dynamics::MetaSkeletonPtr _metaskeleton,
    const aikido::constraint::TestablePtr& _constraint,
    double _dt,
    const VectorFieldCallback& _vectorFieldCb,
    const VectorFieldStatusCallback& _statusCb)
{
  auto saver = MetaSkeletonStateSaver(_metaskeleton);
  DART_UNUSED(saver);

  const auto metaSkeletonProperties = _stateSpace->getProperties();
  const std::size_t numDof = metaSkeletonProperties.getNumDofs();
  const auto positionLowerLimits
      = metaSkeletonProperties.getPositionLowerLimits();
  const auto positionUpperLimits
      = metaSkeletonProperties.getPositionUpperLimits();
  const auto velocityLowerLimits
      = metaSkeletonProperties.getVelocityLowerLimits();
  const auto velocityUpperLimits
      = metaSkeletonProperties.getVelocityUpperLimits();

  if (positionUpperLimits == positionLowerLimits)
  {
    throw std::invalid_argument("State space volume zero");
  }
  if (velocityUpperLimits == velocityLowerLimits)
  {
    throw std::invalid_argument("Velocity space volume zero");
  }

  for (std::size_t i = 0; i < numDof; ++i)
  {
    std::stringstream ss;
    if (positionLowerLimits[i] > positionUpperLimits[i])
    {
      ss << "Position lower limit is larger than upper limit at DOF " << i;
      throw std::invalid_argument(ss.str());
    }
    if (velocityLowerLimits[i] > velocityUpperLimits[i])
    {
      ss << "Velocity lower limit is larger than upper limit at DOF " << i;
      throw std::invalid_argument(ss.str());
    }
  }

  std::vector<Knot> knots;
  VectorFieldPlannerStatus terminationStatus
      = VectorFieldPlannerStatus::CONTINUE;
  std::exception_ptr terminationError;

  int cacheIndex = -1;
  int index = 0;
  double t = 0;
  Eigen::VectorXd q = _metaskeleton->getPositions();
  Eigen::VectorXd dq(numDof);
  assert(static_cast<std::size_t>(q.size()) == numDof);

  do
  {
    // Evaluate the vector field.
    try
    {
      if (!_vectorFieldCb(_stateSpace, _metaskeleton, t, dq))
      {
        throw VectorFieldTerminated("Failed evaluating VectorField.");
      }
    }
    catch (const VectorFieldTerminated& e)
    {
      dtwarn << "Terminating vector field evaluation." << std::endl;
      terminationStatus = VectorFieldPlannerStatus::TERMINATE;
      terminationError = std::current_exception();
      break;
    }

    if (static_cast<std::size_t>(dq.size()) != numDof)
    {
      throw std::length_error(
          "Vector field returned an incorrect number of DOF velocities.");
    }

    // TODO: This should be computed from the DOF resolutions.
    const std::size_t numSteps = 1;

    // Compute the number of collision checks we need.
    for (std::size_t istep = 0; istep < numSteps; ++istep)
    {
      auto currentState = _stateSpace->createState();

      try
      {
        checkDofLimits(_stateSpace, q, dq);
        _stateSpace->convertPositionsToState(q, currentState);
        checkCollision(currentState, _constraint);
      }
      catch (const VectorFieldTerminated& e)
      {
        terminationStatus = VectorFieldPlannerStatus::TERMINATE;
        terminationError = std::current_exception();
        break;
      }

      // Insert the waypoint.
      assert(static_cast<std::size_t>(q.size()) == numDof);
      assert(static_cast<std::size_t>(dq.size()) == numDof);

      Knot knot;
      knot.mT = t;
      knot.mPositions = q;
      knot.mVelocities = dq;
      knots.push_back(knot);

      // Take a step.
      auto deltaState = _stateSpace->createState();
      _stateSpace->convertPositionsToState(_dt * dq, deltaState);

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

  // Print the termination condition.
  if (terminationError)
  {
    try
    {
      std::rethrow_exception(terminationError);
    }
    catch (const VectorFieldTerminated& e)
    {
      dtwarn << "[VectorFieldPlanner::Plan] Terminated early: " << e.what()
             << '\n';
      return nullptr;
    }
  }

  if (cacheIndex >= 0)
  {
    return convertToSpline(knots, cacheIndex, _stateSpace);
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
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& _stateSpace,
    dart::dynamics::MetaSkeletonPtr _metaskeleton,
    const dart::dynamics::BodyNodePtr& _bn,
    const aikido::constraint::TestablePtr& _constraint,
    const Eigen::Vector3d& _direction,
    double _distance,
    double _linearVelocity,
    double _linearTolerance,
    double _angularTolerance,
    double _linearGain,
    double _angularGain,
    double _timestep)
{
  if (_distance < 0.)
  {
    std::stringstream ss;
    ss << "Distance must be non-negative; got " << _distance << ".";
    throw std::runtime_error(ss.str());
  }

  if (_linearVelocity <= 0.0)
  {
    std::stringstream ss;
    ss << "Linear velocity must be positive; got " << _linearVelocity << ".";
    throw std::runtime_error(ss.str());
  }

  if (_direction.norm() == 0.0)
  {
    throw std::runtime_error("Direction vector is a zero vector");
  }

  Eigen::Vector3d velocity = _direction.normalized() * _linearVelocity;
  double duration = _distance / _linearVelocity;

  auto vectorfield = MoveEndEffectorOffsetVectorField(
      _bn,
      velocity,
      0.0,
      duration,
      _timestep,
      _linearGain,
      _linearTolerance,
      _angularGain,
      _angularTolerance);

  return planPathByVectorField(
      _stateSpace,
      _metaskeleton,
      _constraint,
      _timestep,
      vectorfield,
      vectorfield);
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
