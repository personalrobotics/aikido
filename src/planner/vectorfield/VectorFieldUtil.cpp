#include <Eigen/Geometry>
#include <dart/optimizer/Solver.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>
#include <aikido/common/algorithm.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <mcheck.h>

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

protected:
  /// Twist.
  Twist mTwist;

  /// Jacobian of Meta Skeleton.
  Jacobian mJacobian;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}

//==============================================================================
bool computeJointVelocityFromTwist(
    Eigen::VectorXd& jointVelocity,
    const Eigen::Vector6d& desiredTwist,
    const dart::dynamics::MetaSkeletonPtr metaSkeleton,
    const dart::dynamics::ConstBodyNodePtr bodyNode,
    double jointLimitPadding,
    const Eigen::VectorXd& jointVelocityLowerLimits,
    const Eigen::VectorXd& jointVelocityUpperLimits,
    bool enforceJointVelocityLimits,
    double stepSize)
{
  using dart::math::Jacobian;
  using dart::optimizer::Problem;
  using dart::optimizer::Solver;
  using Eigen::VectorXd;

  // Use LBFGS to find joint angles that won't violate the joint limits.
  const Jacobian jacobian = metaSkeleton->getWorldJacobian(bodyNode);

  const std::size_t numDofs = metaSkeleton->getNumDofs();

  jointVelocity = Eigen::VectorXd::Zero(numDofs);
  VectorXd positions = metaSkeleton->getPositions();
  VectorXd initialGuess = metaSkeleton->getVelocities();
  VectorXd positionLowerLimits = metaSkeleton->getPositionLowerLimits();
  VectorXd positionUpperLimits = metaSkeleton->getPositionUpperLimits();
  VectorXd velocityLowerLimits = jointVelocityLowerLimits;
  VectorXd velocityUpperLimits = jointVelocityUpperLimits;

  const auto problem = std::make_shared<Problem>(numDofs);
  if (enforceJointVelocityLimits)
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

  
  const auto problemLocal = std::make_shared<Problem>(numDofs);
  Eigen::Vector6d desiredTwistLocal;
  Eigen::Vector6d initialGuessLocal;
  dart::math::Jacobian jacobianLocal = jacobian;

   desiredTwistLocal << -0.0, 0.0, 0.0, -0.0499474, 0.0742491, -0.995988;
  //  initialGuess.clear();
   initialGuessLocal << -0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // std::cout << "numDofs: " << numDofs << std::endl;
  // std::cout << "cols: " << jacobian.cols() << "   rows: " << jacobian.rows() << std::endl; 
  // std::cout << "cols: " << jacobianLocal.cols() << "   rows: " << jacobianLocal.rows() << std::endl;

  //  jacobian.clear();
   jacobianLocal << -2.07252e-13,    -0.890508,     0.890508,    -0.433307,    -0.782295,   0.00162676,
  1.8255e-16,    -0.454967,     0.454967,     0.848115,     0.369998,  0.000427877,
          -1, -4.71266e-12,  4.64198e-12,    -0.304871,     0.501115,     0.999999,
   -0.355824,   -0.0204959,    -0.154864,    -0.357855,    -0.146731,   -0.0170001,
   -0.229658,    0.0401166,     0.303115,    -0.200778,     -0.29263,  0.000138686,
 7.34577e-14,     0.421351,    -0.281562,   -0.0499281,   -0.0130003,  2.75957e-05;

  problemLocal->setInitialGuess(initialGuessLocal);
  problemLocal->setObjective(
      dart::common::make_aligned_shared<DesiredTwistFunction>(
          desiredTwistLocal, jacobianLocal));

  // dart::optimizer::NloptSolver solverLocal(problemLocal, nlopt::LD_LBFGS);
  dart::optimizer::NloptSolver solverLocal(problemLocal, nlopt::LN_COBYLA);
  // mcheck_check_all();
  solverLocal.solve();


  problem->setInitialGuess(initialGuess);
  problem->setObjective(
      dart::common::make_aligned_shared<DesiredTwistFunction>(
          desiredTwist, jacobian));

  // std::cout << "nlopt initial guess: " << initialGuessLocal.matrix().transpose() << std::endl;
  // std::cout << "nlopt desiredTwist: " << desiredTwistLocal << std::endl;
  // std::cout << "nlopt velocity lower bounds: " << problem->getLowerBounds().matrix().transpose() << std::endl;
  // std::cout << "nlopt velocity upper bounds: " << problem->getUpperBounds().matrix().transpose() << std::endl;
  // std::cout << "nlopt jacobian: " << jacobianLocal.matrix() << std::endl;

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
  Eigen::Vector6d geodesicTwist;
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

} // namespace vectorfield
} // namespace planner
} // namespace aikido
