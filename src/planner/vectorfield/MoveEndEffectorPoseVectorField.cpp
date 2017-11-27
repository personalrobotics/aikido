#include <dart/dynamics/BodyNode.hpp>
#include <dart/math/MathTypes.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorPoseVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

MoveEndEffectorPoseVectorField::MoveEndEffectorPoseVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    dart::dynamics::BodyNodePtr _bn,
    const Eigen::Isometry3d& _goalPose,
    double _poseErrorTolerance,
    double _linearVelocityGain,
    double _angularVelocityGain,
    double _initialStepSize,
    double _jointLimitTolerance,
    double _optimizationTolerance)
  : ConfigurationSpaceVectorField(_stateSpace, _bn)
  , mGoalPose(_goalPose)
  , mPoseErrorTolerance(_poseErrorTolerance)
  , mLinearVelocityGain(_linearVelocityGain)
  , mAngularVelocityGain(_angularVelocityGain)
  , mInitialStepSize(_initialStepSize)
  , mJointLimitTolerance(_jointLimitTolerance)
  , mOptimizationTolerance(_optimizationTolerance)
{
  if (mPoseErrorTolerance < 0)
    throw std::invalid_argument("Pose error tolerance is negative");
  if (mOptimizationTolerance < 0)
    throw std::invalid_argument("Optimization tolerance is negative");

  mVelocityLowerLimits = mMetaSkeleton->getVelocityLowerLimits();
  mVelocityUpperLimits = mMetaSkeleton->getVelocityUpperLimits();
}

//==============================================================================

bool MoveEndEffectorPoseVectorField::getJointVelocities(Eigen::VectorXd& _qd)
{
  using Eigen::Isometry3d;
  using Eigen::Vector3d;
  using Eigen::Vector6d;
  using dart::math::logMap;

  const Isometry3d currentPose = mBodyNode->getTransform();

  Vector6d desiredTwist = computeGeodesicTwist(currentPose, mGoalPose);

  desiredTwist.head<3>() = desiredTwist.head<3>() * mAngularVelocityGain;
  desiredTwist.tail<3>() = desiredTwist.tail<3>() * mLinearVelocityGain;

  Eigen::VectorXd jointVelocityUpperLimits
      = mMetaSkeleton->getVelocityUpperLimits();
  Eigen::VectorXd jointVelocityLowerLimits
      = mMetaSkeleton->getVelocityLowerLimits();

  bool result = computeJointVelocityFromTwist(
      desiredTwist,
      mStateSpace,
      mBodyNode,
      mJointLimitTolerance,
      jointVelocityLowerLimits,
      jointVelocityUpperLimits,
      true,
      mInitialStepSize,
      mOptimizationTolerance,
      _qd);

  if (result == true)
  {
    // Go as fast as possible
    for (std::size_t i = 0; i < mMetaSkeleton->getNumDofs(); i++)
    {
      if (_qd[i] > mVelocityUpperLimits[i])
      {
        _qd[i] = mVelocityUpperLimits[i];
      }
      else if (_qd[i] < mVelocityLowerLimits[i])
      {
        _qd[i] = mVelocityLowerLimits[i];
      }
    }
  }

  return result;
}

//==============================================================================

VectorFieldPlannerStatus MoveEndEffectorPoseVectorField::checkPlanningStatus()
{
  using Eigen::Isometry3d;

  const Isometry3d currentPose = mBodyNode->getTransform();

  double poseError
      = computeGeodesicDistanceBetweenTransforms(currentPose, mGoalPose);

  if (poseError < mPoseErrorTolerance)
  {
    return VectorFieldPlannerStatus::CACHE_AND_TERMINATE;
  }
  return VectorFieldPlannerStatus::CONTINUE;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
