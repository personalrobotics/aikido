#include "aikido/planner/kinodynamic/KinodynamicTimer.hpp"
#include <dart/dart.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/common/StepSequence.hpp>

#include "Path.h"
#include "Trajectory.h"

namespace aikido {
namespace planner {
namespace kinodynamic {

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline>
createSplineFromWaypointsAndConstraints(
    const std::list<Eigen::VectorXd>& waypoints,
    const Eigen::VectorXd& maxVelocities,
    const Eigen::VectorXd& maxAccelerations,
    const Eigen::VectorXd& initialVelocity,
    aikido::statespace::ConstStateSpacePtr stateSpace,
    double maxDeviation,
    double timeStep,
    double startTime)
{
  Trajectory trajectory(
      Path(waypoints, maxDeviation), maxVelocities, maxAccelerations, timeStep);
  if (trajectory.isValid())
  {
    using dart::common::make_unique;
    using CubicSplineProblem = aikido::common::
        SplineProblem<double, int, 4, Eigen::Dynamic, Eigen::Dynamic>;

    std::size_t dimension = stateSpace->getDimension();
    double endTime = startTime + trajectory.getDuration();

    // create spline
    auto outputTrajectory
        = make_unique<aikido::trajectory::Spline>(stateSpace, startTime);

    // create a sequence of time steps from start time to end time by time step
    aikido::common::StepSequence sequence(
        timeStep, true, true, startTime, endTime);

    std::vector<Eigen::VectorXd> positionSeq;
    std::vector<Eigen::VectorXd> velocitySeq;
    for (std::size_t i = 0; i < sequence.getLength(); i++)
    {
      double currT = sequence[i];
      double currTShift = currT - startTime;
      Eigen::VectorXd currentPosition = trajectory.getPosition(currTShift);
      Eigen::VectorXd currentVelocity = trajectory.getVelocity(currTShift);
      positionSeq.push_back(currentPosition);
      velocitySeq.push_back(currentVelocity);
    }

    //preprocess with initial
    std::vector<double> processing(dimension, 0.0);
    for(std::size_t j=0; j<dimension; j++)
    {
      if(std::abs(initialVelocity[j]-velocitySeq[0][j])>0.0)
      {
        // std::cout << "DIM " << j << " IS NON-ZERO INITIAL" << std::endl;
        processing[j] = std::abs(initialVelocity[j])/initialVelocity[j];
      }

      if(std::signbit(initialVelocity[j]-velocitySeq[0][j]) != std::signbit(velocitySeq[1][j]-velocitySeq[0][j]))
      {
        // std::cout << "DIFF SIGN BIT " << std::endl;
        processing[j] = 0.0;
      }
    }
    for(std::size_t j=0; j<dimension; j++)
    {
      if(processing[j]==0.0)
      {
        velocitySeq[0][j] = initialVelocity[j];        
        continue;
      }
      for(std::size_t i=0; i<velocitySeq.size() && (processing[j]!=0.0); i++)
      {
        
        if(initialVelocity[j]*processing[j] > velocitySeq[i][j]*processing[j])
        {
          // std::cout << "UPDATING DIM " << j << " WITH NON-ZERO INITIAL " << initialVelocity[j] << " FROM " << velocitySeq[i][j] << std::endl;
          velocitySeq[i][j] = initialVelocity[j];
        }
        else
        {
          // std::cout << "BECAUSE CURRENT " << velocitySeq[i][j] << " FININSHING UPDATING DIM " << j << std::endl;
          processing[j] = 0.0;
          break;
        }
      }
    }

    const Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(dimension);
    auto currState = stateSpace->createState();
    for (std::size_t i = 0; i < sequence.getLength() - 1; i++)
    {   
      double currT = sequence[i];
      double nextT = sequence[i + 1];
      double segmentDuration = nextT - currT;
      double currTShift = currT - startTime;
      double nextTShift = nextT - startTime;
      Eigen::VectorXd currentPosition = positionSeq[i];
      Eigen::VectorXd nextPosition = positionSeq[i+1]; 
      Eigen::VectorXd currentVelocity = velocitySeq[i];
      Eigen::VectorXd nextVelocity = velocitySeq[i+1];

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

  // path is invalid. Retiming failed.
  return nullptr;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeKinodynamicTiming(
    const aikido::trajectory::Interpolated& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    double maxDeviation,
    double timeStep)
{
  const Eigen::VectorXd initialVelocity
      = Eigen::VectorXd::Zero(maxVelocity.size());
  return computeKinodynamicTiming(
      inputTrajectory,
      maxVelocity,
      maxAcceleration,
      initialVelocity,
      maxDeviation,
      timeStep);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeKinodynamicTiming(
    const aikido::trajectory::Interpolated& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    const Eigen::VectorXd& initialVelocity,
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
  // create waypoints from Interpolated path
  std::list<Eigen::VectorXd> waypoints;
  Eigen::VectorXd tmpVec(stateSpace->getDimension());
  for (std::size_t i = 0; i < inputTrajectory.getNumWaypoints(); i++)
  {
    auto tmpState = inputTrajectory.getWaypoint(i);
    stateSpace->logMap(tmpState, tmpVec);
    waypoints.push_back(tmpVec);
  }
  // Retime waypoints and convert to spline
  return createSplineFromWaypointsAndConstraints(
      waypoints,
      maxVelocity,
      maxAcceleration,
      initialVelocity,
      stateSpace,
      maxDeviation,
      timeStep,
      startTime);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeKinodynamicTiming(
    const aikido::trajectory::Spline& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    double maxDeviation,
    double timeStep)
{
  const Eigen::VectorXd initialVelocity
      = Eigen::VectorXd::Zero(maxVelocity.size());
  return computeKinodynamicTiming(
      inputTrajectory,
      maxVelocity,
      maxAcceleration,
      initialVelocity,
      maxDeviation,
      timeStep);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> computeKinodynamicTiming(
    const aikido::trajectory::Spline& inputTrajectory,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    const Eigen::VectorXd& initialVelocity,
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
  // create waypoints from Spline path
  std::list<Eigen::VectorXd> waypoints;
  Eigen::VectorXd tmpVec(stateSpace->getDimension());
  auto tmpState = stateSpace->createState();
  for (std::size_t i = 0; i < inputTrajectory.getNumWaypoints(); i++)
  {
    inputTrajectory.getWaypoint(i, tmpState);
    stateSpace->logMap(tmpState, tmpVec);
    waypoints.push_back(tmpVec);
  }
  // Retime waypoints and convert to spline
  return createSplineFromWaypointsAndConstraints(
      waypoints,
      maxVelocity,
      maxAcceleration,
      initialVelocity,
      stateSpace,
      maxDeviation,
      timeStep,
      startTime);
}

//==============================================================================
KinodynamicTimer::KinodynamicTimer(
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
std::unique_ptr<aikido::trajectory::Spline> KinodynamicTimer::postprocess(
    const aikido::trajectory::Interpolated& inputTraj,
    const aikido::common::RNG& /*rng*/,
    const aikido::constraint::TestablePtr& /*constraint*/)
{
  return computeKinodynamicTiming(
      inputTraj,
      mVelocityLimits,
      mAccelerationLimits,
      mMaxDeviation,
      mTimeStep);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> KinodynamicTimer::postprocess(
    const aikido::trajectory::Spline& inputTraj,
    const aikido::common::RNG& /*rng*/,
    const aikido::constraint::TestablePtr& /*constraint*/)
{
  return computeKinodynamicTiming(
      inputTraj,
      mVelocityLimits,
      mAccelerationLimits,
      mMaxDeviation,
      mTimeStep);
}

//==============================================================================
const Eigen::VectorXd& KinodynamicTimer::getVelocityLimits() const
{
  return mVelocityLimits;
}

//==============================================================================
const Eigen::VectorXd& KinodynamicTimer::getAccelerationLimits() const
{
  return mAccelerationLimits;
}

//==============================================================================
void KinodynamicTimer::setVelocityLimits(const Eigen::VectorXd& velocityLimits)
{
  mVelocityLimits = velocityLimits;
}

//==============================================================================
void KinodynamicTimer::setAccelerationLimits(
    const Eigen::VectorXd& accelerationLimits)
{
  mAccelerationLimits = accelerationLimits;
}

//==============================================================================
double KinodynamicTimer::getTimeStep() const
{
  return mTimeStep;
}

//==============================================================================
void KinodynamicTimer::setTimeStep(double timeStep)
{
  mTimeStep = timeStep;
}

//==============================================================================
double KinodynamicTimer::getMaxDeviation() const
{
  return mMaxDeviation;
}

//==============================================================================
void KinodynamicTimer::setMaxDeviation(double maxDeviation)
{
  mMaxDeviation = maxDeviation;
}

} // namespace kinodynamic
} // namespace planner
} // namespace aikido
