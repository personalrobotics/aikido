#include <Eigen/Geometry>
#include <aikido/common/algorithm.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& _knots,
    int _cacheIndex,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace)
{
  using dart::common::make_unique;

  std::size_t numDof = _stateSpace->getMetaSkeleton()->getNumDofs();
  // Construct the output spline.
  Eigen::VectorXd times(_cacheIndex);
  std::transform(
      _knots.begin(),
      _knots.begin() + _cacheIndex,
      times.data(),
      [](const Knot& knot) { return knot.mT; });

  auto _outputTrajectory = make_unique<aikido::trajectory::Spline>(_stateSpace);

  using CubicSplineProblem = aikido::common::
      SplineProblem<double, int, 2, Eigen::Dynamic, Eigen::Dynamic>;

  const Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(numDof);
  auto currState = _stateSpace->createState();
  for (int iknot = 0; iknot < _cacheIndex - 1; ++iknot)
  {
    const double segmentDuration = _knots[iknot + 1].mT - _knots[iknot].mT;
    Eigen::VectorXd currentPosition = _knots[iknot].mPositions;
    Eigen::VectorXd nextPosition = _knots[iknot + 1].mPositions;

    if (segmentDuration == 0.0)
    {
      std::cout << "ZERO SEGMENT DURATION " << std::endl;
    }
    CubicSplineProblem problem(Eigen::Vector2d{0., segmentDuration}, 2, numDof);
    problem.addConstantConstraint(0, 0, zeroPosition);
    problem.addConstantConstraint(1, 0, nextPosition - currentPosition);
    const auto solution = problem.fit();
    const auto coefficients = solution.getCoefficients().front();

    _stateSpace->expMap(currentPosition, currState);
    _outputTrajectory->addSegment(coefficients, segmentDuration, currState);
  }
  return _outputTrajectory;
}

//==============================================================================
DesiredTwistFunction::DesiredTwistFunction(
    const Twist& _twist, const Jacobian& _jacobian)
  : dart::optimizer::Function("DesiredTwistFunction")
  , mTwist(_twist)
  , mJacobian(_jacobian)
{
  // Do nothing
}

//==============================================================================
double DesiredTwistFunction::eval(const Eigen::VectorXd& _qd)
{
  return 0.5 * (mJacobian * _qd - mTwist).squaredNorm();
}

//==============================================================================
void DesiredTwistFunction::evalGradient(
    const Eigen::VectorXd& _qd, Eigen::Map<Eigen::VectorXd> _grad)
{
  _grad = mJacobian.transpose() * (mJacobian * _qd - mTwist);
}

//==============================================================================
bool computeJointVelocityFromTwist(
    Eigen::VectorXd& _jointVelocity,
    const Eigen::Vector6d& _desiredTwist,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    const dart::dynamics::BodyNodePtr _bodyNode,
    double _jointLimitPadding,
    const Eigen::VectorXd& _jointVelocityLowerLimits,
    const Eigen::VectorXd& _jointVelocityUpperLimits,
    bool _jointVelocityLimited,
    double _maxStepSize,
    double _optimizationTolerance)
{
  using dart::math::Jacobian;
  using dart::optimizer::Problem;
  using dart::optimizer::Solver;
  using Eigen::VectorXd;

  const dart::dynamics::MetaSkeletonPtr skeleton
      = _stateSpace->getMetaSkeleton();
  // Use LBFGS to find joint angles that won't violate the joint limits.
  const Jacobian jacobian = skeleton->getWorldJacobian(_bodyNode);

  const std::size_t numDofs = skeleton->getNumDofs();

  _jointVelocity = Eigen::VectorXd::Zero(numDofs);
  VectorXd positions = skeleton->getPositions();
  VectorXd initialGuess = skeleton->getVelocities();
  VectorXd positionLowerLimits = skeleton->getPositionLowerLimits();
  VectorXd positionUpperLimits = skeleton->getPositionUpperLimits();
  VectorXd velocityLowerLimits = _jointVelocityLowerLimits;
  VectorXd velocityUpperLimits = _jointVelocityUpperLimits;

  auto currentState = _stateSpace->createState();
  _stateSpace->convertPositionsToState(positions, currentState);

  const auto problem = std::make_shared<Problem>(numDofs);
  if (_jointVelocityLimited)
  {
    for (std::size_t i = 0; i < numDofs; ++i)
    {
      const double position = positions[i];
      const double positionLowerLimit = positionLowerLimits[i];
      const double positionUpperLimit = positionUpperLimits[i];
      const double velocityLowerLimit = velocityLowerLimits[i];
      const double velocityUpperLimit = velocityUpperLimits[i];

      if (position + _maxStepSize * velocityLowerLimit
          <= positionLowerLimit + _jointLimitPadding)
      {
        velocityLowerLimits[i] = 0.0;
      }

      if (position + _maxStepSize * velocityUpperLimit
          >= positionUpperLimit - _jointLimitPadding)
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
      std::make_shared<DesiredTwistFunction>(_desiredTwist, jacobian));

  dart::optimizer::NloptSolver solver(problem, nlopt::LD_LBFGS);
  if (!solver.solve())
  {
    return false;
  }
  double optimalVal = problem->getOptimumValue();
  if (optimalVal > _optimizationTolerance)
  {
    return false;
  }

  _jointVelocity = problem->getOptimalSolution();
  return true;
}

//==============================================================================
Eigen::Vector6d computeGeodesicTwist(
    const Eigen::Isometry3d& _currentTrans, const Eigen::Isometry3d& _goalTrans)
{
  using dart::math::logMap;
  Eigen::Isometry3d relativeTrans = _currentTrans.inverse() * _goalTrans;
  Eigen::Vector3d relativeTranslation
      = _currentTrans.linear() * relativeTrans.translation();
  Eigen::Vector3d axisAngles = logMap(relativeTrans.linear());
  Eigen::Vector3d relativeAngles = _currentTrans.linear() * axisAngles;
  Eigen::Vector6d geodesicTwist;
  geodesicTwist << relativeAngles, relativeTranslation;
  return geodesicTwist;
}

//==============================================================================
Eigen::Vector4d computeGeodesicError(
    const Eigen::Isometry3d& _currentTrans, const Eigen::Isometry3d& _goalTrans)
{
  using dart::math::logMap;
  Eigen::Isometry3d relativeTrans = _currentTrans.inverse() * _goalTrans;
  Eigen::Vector3d relativeTranslation
      = _currentTrans.linear() * relativeTrans.translation();
  Eigen::Vector3d axisAngles = logMap(relativeTrans.linear());
  Eigen::Vector4d geodesicError;
  geodesicError << axisAngles.norm(), relativeTranslation;
  return geodesicError;
}

//==============================================================================
double computeGeodesicDistance(
    const Eigen::Isometry3d& _currentTrans,
    const Eigen::Isometry3d& _goalTrans,
    double _r)
{
  Eigen::Vector4d error = computeGeodesicError(_currentTrans, _goalTrans);
  error[0] = _r * error[0];
  return error.norm();
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
