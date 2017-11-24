#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

MoveEndEffectorOffsetVectorField::MoveEndEffectorOffsetVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    dart::dynamics::BodyNodePtr _bn,
    const Eigen::Vector3d& _direction,
    double _distance,
    double _maxDistance,
    double _positionTolerance,
    double _angularTolerance,
    double _jointLimitTolerance,
    double _optimizationTolerance)
  : ConfigurationSpaceVectorField(_stateSpace, _bn)
  , mDirection(_direction)
  , mDistance(_distance)
  , mMaxDistance(_maxDistance)
  , mPositionTolerance(_positionTolerance)
  , mAngularTolerance(_angularTolerance)
  , mJointLimitTolerance(_jointLimitTolerance)
  , mOptimizationTolerance(_optimizationTolerance)
  , mStartPose(_bn->getTransform())
{
  if (mDistance < 0)
    throw std::invalid_argument("Distance must be non-negative.");
  if (mDirection.norm() == 0)
    throw std::invalid_argument("Direction must be non-zero");
  if (mMaxDistance < mDistance)
    throw std::invalid_argument("Max distance is less than minimum distance.");
  if (mPositionTolerance < 0)
    throw std::invalid_argument("Position tolerance is negative");
  if (mAngularTolerance < 0)
    throw std::invalid_argument("Angular tolerance is negative");
  if (mOptimizationTolerance < 0)
    throw std::invalid_argument("Optimization tolerance is negative");
}

//==============================================================================
bool MoveEndEffectorOffsetVectorField::getJointVelocities(
    const Eigen::VectorXd& _q, double _t, Eigen::VectorXd& _qd)
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::Vector6d;
  using dart::math::logMap;

  const Isometry3d currentPose = mBodyNode->getTransform();

  Vector6d desiredTwist = computeGeodesicTwist(currentPose, mStartPose);
  desiredTwist.tail<3>() = mDirection;

  bool result = computeJointVelocityFromTwist(
      desiredTwist,
      mStateSpace,
      mBodyNode,
      mJointLimitTolerance,
      mOptimizationTolerance,
      _qd);
  return result;
}

//==============================================================================
VectorFieldPlannerStatus MoveEndEffectorOffsetVectorField::checkPlanningStatus(
    const Eigen::VectorXd& _q, double _t)
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::Vector4d;

  const Isometry3d currentPose = mBodyNode->getTransform();

  // Check for deviation from the straight-line trajectory.
  const Vector4d geodesicError = computeGeodesicError(mStartPose, currentPose);
  const double orientationError = geodesicError[0];
  const Vector3d positionError = geodesicError.tail<3>();

  if (fabs(orientationError) > mAngularTolerance)
  {
    throw VectorFieldTerminated("Deviated from orientation constraint.");
  }

  double movedDistance = positionError.transpose()
                         * mDirection; // positionError.dot(mDirection);
  double positionDeviation
      = (positionError - movedDistance * mDirection).norm();
  if (positionDeviation > mPositionTolerance)
  {
    throw VectorFieldTerminated("Deviated from straight line constraint.");
  }

  // Check if we've reached the target.
  if (mMaxDistance == std::numeric_limits<double>::max())
  {
    if (movedDistance > mDistance)
    {
      return VectorFieldPlannerStatus::CACHE_AND_TERMINATE;
    }
  }
  else if (movedDistance > mMaxDistance)
  {
    return VectorFieldPlannerStatus::TERMINATE;
  }
  else if (movedDistance >= mDistance)
  {
    return VectorFieldPlannerStatus::CACHE_AND_CONTINUE;
  }

  return VectorFieldPlannerStatus::CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
