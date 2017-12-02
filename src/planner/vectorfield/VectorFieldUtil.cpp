#include <Eigen/Geometry>
#include <aikido/common/algorithm.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& knots,
    int cacheIndex,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace)
{
  using dart::common::make_unique;

  std::size_t numDof = stateSpace->getMetaSkeleton()->getNumDofs();
  // Construct the output spline.
  Eigen::VectorXd times(cacheIndex);
  std::transform(
      knots.begin(),
      knots.begin() + cacheIndex,
      times.data(),
      [](const Knot& knot) { return knot.mT; });

  auto outputTrajectory = make_unique<aikido::trajectory::Spline>(stateSpace);

  using CubicSplineProblem = aikido::common::
      SplineProblem<double, int, 2, Eigen::Dynamic, Eigen::Dynamic>;

  const Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(numDof);
  auto currState = stateSpace->createState();
  for (int iknot = 0; iknot < cacheIndex - 1; ++iknot)
  {
    const double segmentDuration = knots[iknot + 1].mT - knots[iknot].mT;
    Eigen::VectorXd currentPosition = knots[iknot].mPositions;
    Eigen::VectorXd nextPosition = knots[iknot + 1].mPositions;

    if (segmentDuration == 0.0)
    {
      std::cout << "ZERO SEGMENT DURATION " << std::endl;
    }
    CubicSplineProblem problem(Eigen::Vector2d{0., segmentDuration}, 2, numDof);
    problem.addConstantConstraint(0, 0, zeroPosition);
    problem.addConstantConstraint(1, 0, nextPosition - currentPosition);
    const auto solution = problem.fit();
    const auto coefficients = solution.getCoefficients().front();

    stateSpace->expMap(currentPosition, currState);
    outputTrajectory->addSegment(coefficients, segmentDuration, currState);
  }
  return outputTrajectory;
}

//==============================================================================
DesiredTwistFunction::DesiredTwistFunction(
    const Twist& twist, const Jacobian& jacobian)
  : dart::optimizer::Function("DesiredTwistFunction")
  , mTwist(twist)
  , mJacobian(jacobian)
{
  // Do nothing
}

//==============================================================================
double DesiredTwistFunction::eval(const Eigen::VectorXd& qd)
{
  return 0.5 * (mJacobian * qd - mTwist).squaredNorm();
}

//==============================================================================
void DesiredTwistFunction::evalGradient(
    const Eigen::VectorXd& qd, Eigen::Map<Eigen::VectorXd> grad)
{
  grad = mJacobian.transpose() * (mJacobian * qd - mTwist);
}

//==============================================================================
bool computeJointVelocityFromTwist(
    Eigen::VectorXd& jointVelocity,
    const Eigen::Vector6d& desiredTwist,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const dart::dynamics::BodyNodePtr bodyNode,
    double jointLimitPadding,
    const Eigen::VectorXd& jointVelocityLowerLimits,
    const Eigen::VectorXd& jointVelocityUpperLimits,
    bool jointVelocityLimited,
    double maxStepSize,
    double optimizationTolerance)
{
  using dart::math::Jacobian;
  using dart::optimizer::Problem;
  using dart::optimizer::Solver;
  using Eigen::VectorXd;

  const dart::dynamics::MetaSkeletonPtr skeleton
      = stateSpace->getMetaSkeleton();
  // Use LBFGS to find joint angles that won't violate the joint limits.
  const Jacobian jacobian = skeleton->getWorldJacobian(bodyNode);

  const std::size_t numDofs = skeleton->getNumDofs();

  jointVelocity = Eigen::VectorXd::Zero(numDofs);
  VectorXd positions = skeleton->getPositions();
  VectorXd initialGuess = skeleton->getVelocities();
  VectorXd positionLowerLimits = skeleton->getPositionLowerLimits();
  VectorXd positionUpperLimits = skeleton->getPositionUpperLimits();
  VectorXd velocityLowerLimits = jointVelocityLowerLimits;
  VectorXd velocityUpperLimits = jointVelocityUpperLimits;

  auto currentState = stateSpace->createState();
  stateSpace->convertPositionsToState(positions, currentState);

  const auto problem = std::make_shared<Problem>(numDofs);
  if (jointVelocityLimited)
  {
    for (std::size_t i = 0; i < numDofs; ++i)
    {
      const double position = positions[i];
      const double positionLowerLimit = positionLowerLimits[i];
      const double positionUpperLimit = positionUpperLimits[i];
      const double velocityLowerLimit = velocityLowerLimits[i];
      const double velocityUpperLimit = velocityUpperLimits[i];

      if (position + maxStepSize * velocityLowerLimit
          <= positionLowerLimit + jointLimitPadding)
      {
        velocityLowerLimits[i] = 0.0;
      }

      if (position + maxStepSize * velocityUpperLimit
          >= positionUpperLimit - jointLimitPadding)
      {
        velocityUpperLimits[i] = 0.0;
      }

      initialGuess[i] = common::clamp(
          initialGuess[i], velocityLowerLimits[i], velocityUpperLimits[i]);
    }

    problem->setLowerBounds(velocityLowerLimits);
    problem->setUpperBounds(velocityUpperLimits);
  }
  problem->setInitialGuess(initialGuess);
  problem->setObjective(
      std::make_shared<DesiredTwistFunction>(desiredTwist, jacobian));

  dart::optimizer::NloptSolver solver(problem, nlopt::LD_LBFGS);
  if (!solver.solve())
  {
    return false;
  }
  double optimalVal = problem->getOptimumValue();
  if (optimalVal > optimizationTolerance)
  {
    return false;
  }

  jointVelocity = problem->getOptimalSolution();
  return true;
}

//==============================================================================
Eigen::Vector6d computeGeodesicTwist(
    const Eigen::Isometry3d& currentTrans, const Eigen::Isometry3d& goalTrans)
{
  using dart::math::logMap;
  Eigen::Isometry3d relativeTrans = currentTrans.inverse() * goalTrans;
  Eigen::Vector3d relativeTranslation
      = currentTrans.linear() * relativeTrans.translation();
  Eigen::Vector3d axisAngles = logMap(relativeTrans.linear());
  Eigen::Vector3d relativeAngles = currentTrans.linear() * axisAngles;
  Eigen::Vector6d geodesicTwist = Eigen::Vector6d::Zero();
  geodesicTwist << relativeAngles, relativeTranslation;
  return geodesicTwist;
}

//==============================================================================
Eigen::Vector4d computeGeodesicError(
    const Eigen::Isometry3d& currentTrans, const Eigen::Isometry3d& goalTrans)
{
  using dart::math::logMap;
  Eigen::Isometry3d relativeTrans = currentTrans.inverse() * goalTrans;
  Eigen::Vector3d relativeTranslation
      = currentTrans.linear() * relativeTrans.translation();
  Eigen::Vector3d axisAngles = logMap(relativeTrans.linear());
  Eigen::Vector4d geodesicError;
  geodesicError << axisAngles.norm(), relativeTranslation;
  return geodesicError;
}

//==============================================================================
double computeGeodesicDistance(
    const Eigen::Isometry3d& currentTrans,
    const Eigen::Isometry3d& goalTrans,
    double r)
{
  Eigen::Vector4d error = computeGeodesicError(currentTrans, goalTrans);
  error[0] = r * error[0];
  return error.norm();
}

//==============================================================================
bool getTransfromFromTimedSE3Trajectory(
    std::shared_ptr<aikido::statespace::SE3> stateSpace,
    const aikido::trajectory::Interpolated* timedSE3Path,
    double t,
    Eigen::Isometry3d& trans)
{
  if (t < timedSE3Path->getStartTime() || t > timedSE3Path->getEndTime())
  {
    return false;
  }
  auto state = stateSpace->createState();
  timedSE3Path->evaluate(t, state);
  trans = stateSpace->getIsometry(state);
  return true;
}

//==============================================================================
double getEuclideanDistanceBetweenTransforms(
    const Eigen::Isometry3d& currentTrans, const Eigen::Isometry3d& goalTrans)
{
  Eigen::Vector3d currentVec = currentTrans.translation();
  Eigen::Vector3d goalVec = goalTrans.translation();
  Eigen::Vector3d deltaVec = currentVec - goalVec;
  double distance = deltaVec.transpose() * deltaVec;
  return std::sqrt(distance);
}

//==============================================================================
double getErrorFromTimedSE3Trajectory(
    const Eigen::Isometry3d& currentTrans,
    const aikido::trajectory::Interpolated* timedSE3Path,
    std::shared_ptr<aikido::statespace::SE3> stateSpace,
    double t)

{
  Eigen::Isometry3d nearestTrans = Eigen::Isometry3d::Identity();
  if (getTransfromFromTimedSE3Trajectory(
          stateSpace, timedSE3Path, t, nearestTrans))
  {
    return getEuclideanDistanceBetweenTransforms(currentTrans, nearestTrans);
  }

  return false;
}

//==============================================================================
bool getMinDistanceBetweenTransformAndWorkspaceTraj(
    const Eigen::Isometry3d& currentPose,
    const aikido::trajectory::Interpolated* timedWorkspacePath,
    const std::shared_ptr<aikido::statespace::SE3> SE3StateSpace,
    double dt,
    double& minDist,
    double& tLoc,
    Eigen::Isometry3d& transLoc)
{
  minDist = std::numeric_limits<double>::max();
  tLoc = 0.0;

  // Iterate over the trajectory
  double t = 0.0;
  double duration = timedWorkspacePath->getDuration();
  while (t <= duration)
  {
    double error = getErrorFromTimedSE3Trajectory(
        currentPose, timedWorkspacePath, SE3StateSpace, t);
    if (error < minDist)
    {
      minDist = error;
      tLoc = t;
    }
    t = t + dt;
  }

  return getTransfromFromTimedSE3Trajectory(
      SE3StateSpace, timedWorkspacePath, tLoc, transLoc);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Interpolated>
timeTrajectoryByGeodesicUnitTiming(
    const aikido::trajectory::Interpolated* untimedTraj,
    const std::shared_ptr<aikido::statespace::SE3> SE3StateSpace,
    double alpha)
{
  using dart::common::make_unique;
  using std::make_shared;
  using aikido::trajectory::Interpolated;
  using aikido::statespace::Interpolator;

  std::size_t numWaypoints = untimedTraj->getNumWaypoints();
  if (numWaypoints <= 1)
  {
    dtwarn << "Trajectory needs more than 1 waypoint." << std::endl;
    return nullptr;
  }

  // Create a new workspace trajectory with the same spec as the old one
  std::shared_ptr<Interpolator> interpolator
      = make_shared<aikido::statespace::GeodesicInterpolator>(SE3StateSpace);
  auto outputTrajectory = make_unique<aikido::trajectory::Interpolated>(
      SE3StateSpace, interpolator);

  // Get the current pose of the end effector
  auto currentState = untimedTraj->getWaypoint(0);
  auto currentSE3State
      = static_cast<const aikido::statespace::SE3::State*>(currentState);
  Eigen::Isometry3d currentTrans = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d nextTrans = Eigen::Isometry3d::Identity();
  currentTrans = SE3StateSpace->getIsometry(currentSE3State);

  double currentTime = 0.0;
  outputTrajectory->addWaypoint(currentTime, currentState);
  for (std::size_t i = 1; i < numWaypoints; i++)
  {
    auto nextState = untimedTraj->getWaypoint(i);
    auto nextSE3State
        = static_cast<const aikido::statespace::SE3::State*>(nextState);
    nextTrans = SE3StateSpace->getIsometry(nextSE3State);
    double dist = computeGeodesicDistance(currentTrans, nextTrans, alpha);

    currentTime += dist;
    currentTrans = nextTrans;
    currentState = nextState;

    outputTrajectory->addWaypoint(currentTime, currentState);
  }

  return outputTrajectory;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
