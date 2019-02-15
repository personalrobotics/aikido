#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include <dart/dart.hpp>
#include "aikido/common/Spline.hpp"
#include "aikido/common/StepSequence.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/trajectory/util.hpp"

#include "Path.h"
#include "Trajectory.h"

using dart::common::make_unique;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::ConstStateSpacePtr;
using aikido::trajectory::toR1JointTrajectory;
using aikido::trajectory::ConstSplinePtr;
using aikido::trajectory::ConstInterpolatedPtr;

namespace aikido {
namespace planner {
namespace kunzretimer {

namespace detail {
//==============================================================================
std::unique_ptr<Path> convertToKunzPath(
    const aikido::trajectory::Interpolated& traj, double maxDeviation)
{
  auto trajectory = toR1JointTrajectory(traj);
  auto stateSpace = trajectory->getStateSpace();

  std::list<Eigen::VectorXd> waypoints;
  Eigen::VectorXd tmpVec(stateSpace->getDimension());
  for (std::size_t i = 0; i < trajectory->getNumWaypoints(); i++)
  {
    auto tmpState = trajectory->getWaypoint(i);
    stateSpace->logMap(tmpState, tmpVec);
    waypoints.push_back(tmpVec);
  }
  auto path = make_unique<Path>(waypoints, maxDeviation);
  return path;
}

//==============================================================================
std::unique_ptr<Path> convertToKunzPath(
    const aikido::trajectory::Spline& traj, double maxDeviation)
{
  auto trajectory = toR1JointTrajectory(traj);
  auto stateSpace = trajectory->getStateSpace();

  std::list<Eigen::VectorXd> waypoints;

  stateSpace = trajectory->getStateSpace();
  Eigen::VectorXd tmpVec(stateSpace->getDimension());
  auto tmpState = stateSpace->createState();
  for (std::size_t i = 0; i < trajectory->getNumWaypoints(); i++)
  {
    trajectory->getWaypoint(i, tmpState);
    stateSpace->logMap(tmpState, tmpVec);
    waypoints.push_back(tmpVec);
  }

  auto path = make_unique<Path>(waypoints, maxDeviation);
  return path;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const Trajectory& traj,
    aikido::statespace::ConstStateSpacePtr stateSpace,
    double timeStep,
    double startTime)
{
  using CubicSplineProblem = aikido::common::
      SplineProblem<double, int, 4, Eigen::Dynamic, Eigen::Dynamic>;

  std::size_t dimension = stateSpace->getDimension();
  double endTime = startTime + traj.getDuration();

  // create spline
  auto outputTrajectory
      = ::dart::common::make_unique<aikido::trajectory::Spline>(
          stateSpace, startTime);

  // create a sequence of time steps from start time to end time by time step
  aikido::common::StepSequence sequence(
      timeStep, true, true, startTime, endTime);

  const Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(dimension);
  auto currState = stateSpace->createState();
  for (std::size_t i = 0; i < sequence.getLength() - 1; i++)
  {
    double currT = sequence[i];
    double nextT = sequence[i + 1];
    double segmentDuration = nextT - currT;
    double currTShift = currT - startTime;
    double nextTShift = nextT - startTime;
    Eigen::VectorXd currentPosition = traj.getPosition(currTShift);
    Eigen::VectorXd nextPosition = traj.getPosition(nextTShift);
    Eigen::VectorXd currentVelocity = traj.getVelocity(currTShift);
    Eigen::VectorXd nextVelocity = traj.getVelocity(nextTShift);

    CubicSplineProblem problem(
        Eigen::Vector2d{0., segmentDuration}, 4, dimension);
    problem.addConstantConstraint(0, 0, zeroPosition);
    problem.addConstantConstraint(0, 1, currentVelocity);
    problem.addConstantConstraint(1, 0, nextPosition - currentPosition);
    problem.addConstantConstraint(1, 1, nextVelocity);
    const auto solution = problem.fit();
    const auto coefficients = solution.getCoefficients().front();

    stateSpace->expMap(currentPosition, currState);
    outputTrajectory->addSegment(coefficients, segmentDuration, currState);
  }

  return outputTrajectory;
}

} // namespace detail

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeKunzTiming(
    const aikido::trajectory::Interpolated& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    double maxDeviation,
    double timeStep)
{
  const auto stateSpace = inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();

  if (static_cast<std::size_t>(maxVelocity.size()) != dimension)
    throw std::invalid_argument("Velocity limits have wrong dimension.");

  if (static_cast<std::size_t>(maxAcceleration.size()) != dimension)
    throw std::invalid_argument("Acceleration limits have wrong dimension.");

  for (std::size_t i = 0; i < dimension; ++i)
  {
    if (maxVelocity[i] <= 0.)
      throw std::invalid_argument("Velocity limits must be positive.");
    if (!std::isfinite(maxVelocity[i]))
      throw std::invalid_argument("Velocity limits must be finite.");

    if (maxAcceleration[i] <= 0.)
      throw std::invalid_argument("Acceleration limits must be positive.");
    if (!std::isfinite(maxAcceleration[i]))
      throw std::invalid_argument("Acceleration limits must be finite.");
  }

  double startTime = inputTrajectory.getStartTime();
  auto path = detail::convertToKunzPath(inputTrajectory, maxDeviation);
  Trajectory trajectory(*path, maxVelocity, maxAcceleration, timeStep);
  return detail::convertToSpline(trajectory, stateSpace, timeStep, startTime);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeKunzTiming(
    const aikido::trajectory::Spline& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    double maxDeviation,
    double timeStep)
{
  const auto stateSpace = inputTrajectory.getStateSpace();
  const auto dimension = stateSpace->getDimension();

  if (static_cast<std::size_t>(maxVelocity.size()) != dimension)
    throw std::invalid_argument("Velocity limits have wrong dimension.");

  if (static_cast<std::size_t>(maxAcceleration.size()) != dimension)
    throw std::invalid_argument("Acceleration limits have wrong dimension.");

  for (std::size_t i = 0; i < dimension; ++i)
  {
    if (maxVelocity[i] <= 0.)
      throw std::invalid_argument("Velocity limits must be positive.");
    if (!std::isfinite(maxVelocity[i]))
      throw std::invalid_argument("Velocity limits must be finite.");

    if (maxAcceleration[i] <= 0.)
      throw std::invalid_argument("Acceleration limits must be positive.");
    if (!std::isfinite(maxAcceleration[i]))
      throw std::invalid_argument("Acceleration limits must be finite.");
  }

  double startTime = inputTrajectory.getStartTime();
  auto path = detail::convertToKunzPath(inputTrajectory, maxDeviation);
  Trajectory trajectory(*path, maxVelocity, maxAcceleration, timeStep);
  return detail::convertToSpline(trajectory, stateSpace, timeStep, startTime);
}

//==============================================================================
KunzRetimer::KunzRetimer(
    const Eigen::VectorXd& velocityLimits,
    const Eigen::VectorXd& accelerationLimits,
    double maxDeviation,
    double timeStep)
  : mVelocityLimits{velocityLimits}
  , mAccelerationLimits{accelerationLimits}
  , mMaxDeviation(maxDeviation)
  , mTimeStep(timeStep)
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> KunzRetimer::postprocess(
    const aikido::trajectory::Interpolated& inputTraj,
    const aikido::common::RNG& /*rng*/,
    const aikido::constraint::TestablePtr& /*constraint*/)
{
  return computeKunzTiming(
      inputTraj,
      mVelocityLimits,
      mAccelerationLimits,
      mMaxDeviation,
      mTimeStep);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> KunzRetimer::postprocess(
    const aikido::trajectory::Spline& inputTraj,
    const aikido::common::RNG& /*rng*/,
    const aikido::constraint::TestablePtr& /*constraint*/)
{
  return computeKunzTiming(
      inputTraj,
      mVelocityLimits,
      mAccelerationLimits,
      mMaxDeviation,
      mTimeStep);
}

//==============================================================================
const Eigen::VectorXd& KunzRetimer::getVelocityLimits() const
{
  return mVelocityLimits;
}

//==============================================================================
const Eigen::VectorXd& KunzRetimer::getAccelerationLimits() const
{
  return mAccelerationLimits;
}

//==============================================================================
void KunzRetimer::setVelocityLimits(const Eigen::VectorXd& velocityLimits)
{
  mVelocityLimits = velocityLimits;
}

//==============================================================================
void KunzRetimer::setAccelerationLimits(
    const Eigen::VectorXd& accelerationLimits)
{
  mAccelerationLimits = accelerationLimits;
}

//==============================================================================
double KunzRetimer::getTimeStep() const
{
  return mTimeStep;
}

//==============================================================================
void KunzRetimer::setTimeStep(double timeStep)
{
  mTimeStep = timeStep;
}

//==============================================================================
double KunzRetimer::getMaxDeviation() const
{
  return mMaxDeviation;
}

//==============================================================================
void KunzRetimer::setMaxDeviation(double maxDeviation)
{
  mMaxDeviation = maxDeviation;
}

} // namespace kunzretimer
} // namespace planner
} // namespace aikido
