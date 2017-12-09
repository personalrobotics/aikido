#include <Eigen/Geometry>
#include <dart/optimizer/Solver.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>
#include <aikido/common/algorithm.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

namespace {

/// A function class that defines an objective. The objective measures
/// the difference between a desired twist and Jacobian * joint velocities.
///
class DesiredTwistFunction : public dart::optimizer::Function
{
public:
  using Twist = Eigen::Vector6d;
  using Jacobian = dart::math::Jacobian;

  /// Constructor.
  ///
  /// \param[in] twist A desired twist.
  /// \param[in] jacobian System Jacobian.
  DesiredTwistFunction(const Twist& twist, const Jacobian& jacobian)
    : dart::optimizer::Function("DesiredTwistFunction")
    , mTwist(twist)
    , mJacobian(jacobian)
  {
    // Do nothing
  }

  /// Implementation inherited.
  /// Evaluating an objective by a state value.
  ///
  /// \param[in] qd Joint velocities.
  /// \return Objective value.
  double eval(const Eigen::VectorXd& qd) override
  {
    return 0.5 * (mJacobian * qd - mTwist).squaredNorm();
  }

  /// Implementation inherited.
  /// Evaluating gradient of an objective by a state value.
  /// \param[in] qd Joint velocities.
  /// \param[out] grad Gradient of a defined objective.
  void evalGradient(
      const Eigen::VectorXd& qd, Eigen::Map<Eigen::VectorXd> grad) override
  {
    grad = mJacobian.transpose() * (mJacobian * qd - mTwist);
  }

private:
  /// Twist.
  Twist mTwist;

  /// Jacobian of Meta Skeleton.
  Jacobian mJacobian;
};
}

//==============================================================================
bool computeJointVelocityFromTwist(
    Eigen::VectorXd& jointVelocity,
    const Eigen::Vector6d& desiredTwist,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const dart::dynamics::BodyNodePtr bodyNode,
    double jointLimitPadding,
    const Eigen::VectorXd* jointVelocityLowerLimits,
    const Eigen::VectorXd* jointVelocityUpperLimits,
    double stepSize)
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
  auto currentState = stateSpace->createState();
  stateSpace->convertPositionsToState(positions, currentState);

  const auto problem = std::make_shared<Problem>(numDofs);
  if (jointVelocityLowerLimits != nullptr)
  {
    VectorXd velocityLowerLimits = *jointVelocityLowerLimits;
    for (std::size_t i = 0; i < numDofs; ++i)
    {
      const double position = positions[i];
      const double positionLowerLimit = positionLowerLimits[i];
      const double velocityLowerLimit = velocityLowerLimits[i];

      if (position + stepSize * velocityLowerLimit
          <= positionLowerLimit + jointLimitPadding)
      {
        velocityLowerLimits[i] = 0.0;
      }

      if (initialGuess[i] < velocityLowerLimits[i])
      {
        initialGuess[i] = velocityLowerLimits[i];
      }
    }
    problem->setLowerBounds(velocityLowerLimits);
  }

  if (jointVelocityUpperLimits == nullptr)
  {
    VectorXd velocityUpperLimits = *jointVelocityUpperLimits;
    for (std::size_t i = 0; i < numDofs; ++i)
    {
      const double position = positions[i];
      const double positionUpperLimit = positionUpperLimits[i];
      const double velocityUpperLimit = velocityUpperLimits[i];

      if (position + stepSize * velocityUpperLimit
          >= positionUpperLimit - jointLimitPadding)
      {
        velocityUpperLimits[i] = 0.0;
      }

      if (initialGuess[i] > velocityUpperLimits[i])
      {
        initialGuess[i] = velocityUpperLimits[i];
      }
    }
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
  jointVelocity = problem->getOptimalSolution();
  return true;
}

//==============================================================================
Eigen::Vector6d computeGeodesicTwist(
    const Eigen::Isometry3d& fromTrans, const Eigen::Isometry3d& toTrans)
{
  using dart::math::logMap;
  Eigen::Isometry3d relativeTrans = fromTrans.inverse() * toTrans;
  Eigen::Vector3d relativeTranslation
      = fromTrans.linear() * relativeTrans.translation();
  Eigen::Vector3d axisAngles = logMap(relativeTrans.linear());
  Eigen::Vector3d relativeAngles = fromTrans.linear() * axisAngles;
  Eigen::Vector6d geodesicTwist = Eigen::Vector6d::Zero();
  geodesicTwist << relativeAngles, relativeTranslation;
  return geodesicTwist;
}

//==============================================================================
Eigen::Vector4d computeGeodesicError(
    const Eigen::Isometry3d& fromTrans, const Eigen::Isometry3d& toTrans)
{
  using dart::math::logMap;
  Eigen::Isometry3d relativeTrans = fromTrans.inverse() * toTrans;
  Eigen::Vector3d relativeTranslation
      = fromTrans.linear() * relativeTrans.translation();
  Eigen::Vector3d axisAngles = logMap(relativeTrans.linear());
  Eigen::Vector4d geodesicError;
  geodesicError << axisAngles.norm(), relativeTranslation;
  return geodesicError;
}

//==============================================================================
double computeGeodesicDistance(
    const Eigen::Isometry3d& fromTrans,
    const Eigen::Isometry3d& toTrans,
    double r)
{
  Eigen::Vector4d error = computeGeodesicError(fromTrans, toTrans);
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
