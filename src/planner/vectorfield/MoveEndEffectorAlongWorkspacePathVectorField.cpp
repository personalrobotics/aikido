#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorAlongWorkspacePathVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

constexpr double normThreshold = 1e-10;
constexpr double trajectoryCheckStepSize = 0.0005;

//==============================================================================
MoveEndEffectorAlongWorkspacePathVectorField::
    MoveEndEffectorAlongWorkspacePathVectorField(
        aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
        dart::dynamics::BodyNodePtr bn,
        aikido::trajectory::InterpolatedPtr workspacePath,
        double positionTolerance,
        double angularTolerance,
        double tStep,
        double initialStepSize,
        double jointLimitPadding,
        double optimizationTolerance,
        const Eigen::Vector6d& kpFF,
        const Eigen::Vector6d& kpE)
  : ConfigurationSpaceVectorField(stateSpace, bn)
  , mWorkspacePath(workspacePath)
  , mPositionTolerance(positionTolerance)
  , mAngularTolerance(angularTolerance)
  , mDeltaT(tStep)
  , mInitialStepSize(initialStepSize)
  , mJointLimitPadding(jointLimitPadding)
  , mOptimizationTolerance(optimizationTolerance)
  , mKpFF(kpFF)
  , mKpE(kpE)
  , mStartPose(bn->getTransform())
{
  if (mPositionTolerance < 0)
    throw std::invalid_argument("Position tolerance is negative");
  if (mAngularTolerance < 0)
    throw std::invalid_argument("Angular tolerance is negative");
  if (mOptimizationTolerance < 0)
    throw std::invalid_argument("Optimization tolerance is negative");

  mSE3StateSpace = std::make_shared<aikido::statespace::SE3>();
  // Time the trajectory based on its distance
  mTimedWorkspacePath = timeTrajectoryByGeodesicUnitTiming(
      mWorkspacePath.get(), mSE3StateSpace);

  mDuration = mTimedWorkspacePath->getDuration();
  auto endState = mSE3StateSpace->createState();
  mTimedWorkspacePath->evaluate(mTimedWorkspacePath->getEndTime(), endState);
  mSE3StateSpace->setIsometry(endState, mGoalPose);
}

//==============================================================================
bool MoveEndEffectorAlongWorkspacePathVectorField::getJointVelocities(
    Eigen::VectorXd& qd) const
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::Vector6d;
  using dart::math::logMap;

  const Isometry3d actualTEE = mBodyNode->getTransform();

  // Find where we are on the goal trajectory by finding the the closest point
  double minDist = 0.0;
  double t = 0.0;
  Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
  getMinDistanceBetweenTransformAndWorkspaceTraj(
      actualTEE,
      mTimedWorkspacePath.get(),
      mSE3StateSpace,
      trajectoryCheckStepSize,
      minDist,
      t,
      trans);

  // Get the desired end-effector transform from the goal trajectory
  Eigen::Isometry3d desiredTEE = Eigen::Isometry3d::Identity();
  getTransfromFromTimedSE3Trajectory(
      mSE3StateSpace, mTimedWorkspacePath.get(), t, desiredTEE);

  // Get the next end-effector transform, using finite-differences
  Eigen::Isometry3d desiredTEEnext = Eigen::Isometry3d::Identity();
  getTransfromFromTimedSE3Trajectory(
      mSE3StateSpace, mTimedWorkspacePath.get(), t + mDeltaT, desiredTEEnext);

  // Get the translation tangent to current poisition
  Eigen::Vector3d tangentVec
      = desiredTEEnext.translation() - desiredTEE.translation();

  // Get the translational error
  Eigen::Vector3d positionErrorVec
      = desiredTEE.translation() - actualTEE.translation();

  // Get the translational error perpendicular to the path
  Eigen::Vector3d normalizedTangentVec = tangentVec.normalized();
  Eigen::Vector3d tangentTransError
      = positionErrorVec
        - positionErrorVec.dot(normalizedTangentVec) * normalizedTangentVec;

  // The twist between the actual end-effector position and where it should be
  // on the goal trajectory (the error term)
  Eigen::Vector6d twistPerpendicular
      = computeGeodesicTwist(actualTEE, desiredTEE);
  twistPerpendicular.tail<3>() = tangentTransError;

  // The twist tangent to where the end-effector should be on the goal
  // trajectory (the feed-forward term)
  Eigen::Vector6d twistParallel
      = computeGeodesicTwist(desiredTEE, desiredTEEnext);

  // Normalize the translational and angular velocity of the feed-forward twist
  if (twistParallel.head<3>().norm() > normThreshold)
    twistParallel.head<3>().normalize();
  if (twistParallel.head<3>().norm() > normThreshold)
    twistParallel.tail<3>().normalize();

  // Apply gains
  Vector6d desiredTwist = Eigen::Vector6d::Zero();
  for (std::size_t i = 0; i < 6; i++)
  {
    desiredTwist[i]
        = mKpE[i] * twistPerpendicular[i] + mKpFF[i] * twistParallel[i];
  }

  Eigen::VectorXd jointVelocityUpperLimits
      = mMetaSkeleton->getVelocityUpperLimits();
  Eigen::VectorXd jointVelocityLowerLimits
      = mMetaSkeleton->getVelocityLowerLimits();

  // Calculate joint velocities using an optimized jacobian
  bool result = computeJointVelocityFromTwist(
      qd,
      desiredTwist,
      mStateSpace,
      mBodyNode,
      mJointLimitPadding,
      jointVelocityLowerLimits,
      jointVelocityUpperLimits,
      true,
      mInitialStepSize,
      mOptimizationTolerance);
  return result;
}

//==============================================================================
VectorFieldPlannerStatus
MoveEndEffectorAlongWorkspacePathVectorField::checkPlanningStatus() const
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::Vector4d;

  const Isometry3d currTEE = mBodyNode->getTransform();

  // Find where we are on the goal trajectory by finding the closest point
  double minDist = 0.0;
  double t = 0.0;
  Eigen::Isometry3d trans;
  getMinDistanceBetweenTransformAndWorkspaceTraj(
      currTEE,
      mTimedWorkspacePath.get(),
      mSE3StateSpace,
      trajectoryCheckStepSize,
      minDist,
      t,
      trans);

  // Get the desired end-effector transform from the goal trajectory
  Eigen::Isometry3d desiredTEE = Eigen::Isometry3d::Identity();
  getTransfromFromTimedSE3Trajectory(
      mSE3StateSpace, mTimedWorkspacePath.get(), t, desiredTEE);

  // Get the position vector tangent to the trajectory, using finite-differences
  Eigen::Isometry3d desiredTEEnext;
  getTransfromFromTimedSE3Trajectory(
      mSE3StateSpace, mTimedWorkspacePath.get(), t + mDeltaT, desiredTEEnext);
  Eigen::Vector3d tangentVec
      = desiredTEEnext.translation() - desiredTEE.translation();

  // Calculate error between current end-effector pose and where we should be on
  // the goal trajectory
  Eigen::Vector6d geodesicError = computeGeodesicTwist(desiredTEE, currTEE);
  Eigen::Vector3d orientationError = geodesicError.tail<3>();
  Eigen::Vector3d positionErrorVec = geodesicError.head<3>();

  // Use only the translation error that is perpendicular to our current
  // position
  Eigen::Vector3d normalizedTangentVec = tangentVec.normalized();
  Eigen::Vector3d tangentTransError
      = positionErrorVec
        - positionErrorVec.dot(normalizedTangentVec) * normalizedTangentVec;
  Eigen::Vector3d positionError = tangentTransError;

  if (orientationError.norm() > mAngularTolerance)
  {
    throw VectorFieldTerminated("Deviated from orientation constraint.");
  }

  double positionDeviation = positionError.norm();
  if (positionDeviation > mPositionTolerance)
  {
    throw VectorFieldTerminated("Deviated from straight line constraint.");
  }

  // Check if we have reached the end of the goal trajectory
  Eigen::Vector4d errorToGoal = computeGeodesicError(currTEE, mGoalPose);
  double orientationErrorToGoal = errorToGoal[3];
  Eigen::Vector3d positionErrorToGoal = errorToGoal.tail<3>();
  double positionErrorToGoalNorm = positionErrorToGoal.norm();

  if (orientationErrorToGoal < mAngularTolerance
      && positionErrorToGoalNorm < mPositionTolerance)
  {
    return VectorFieldPlannerStatus::CACHE_AND_TERMINATE;
  }

  return VectorFieldPlannerStatus::CACHE_AND_CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
