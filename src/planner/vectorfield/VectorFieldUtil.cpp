#include <Eigen/Geometry>
#include <aikido/common/algorithm.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

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

      if (position + stepSize * velocityLowerLimit
          <= positionLowerLimit + jointLimitPadding)
      {
        velocityLowerLimits[i] = 0.0;
      }

      if (position + stepSize * velocityUpperLimit
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
  Eigen::Vector6d geodesicTwist;
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

} // namespace vectorfield
} // namespace planner
} // namespace aikido
