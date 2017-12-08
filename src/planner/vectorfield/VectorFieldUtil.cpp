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

  std::size_t numDof = _stateSpace->getProperties().getNumDofs();
  // Construct the output spline.
  Eigen::VectorXd times(_cacheIndex);
  std::transform(
      _knots.begin(),
      _knots.begin() + _cacheIndex,
      times.data(),
      [](const Knot& knot) { return knot.mT; });

  auto _outputTrajectory = make_unique<aikido::trajectory::Spline>(_stateSpace);

  using CubicSplineProblem = aikido::common::
      SplineProblem<double, int, 4, Eigen::Dynamic, Eigen::Dynamic>;

  const Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(numDof);
  auto currState = _stateSpace->createState();
  for (int iknot = 0; iknot < _cacheIndex - 1; ++iknot)
  {
    const double segmentDuration = _knots[iknot + 1].mT - _knots[iknot].mT;
    Eigen::VectorXd currentPosition = _knots[iknot].mPositions;
    Eigen::VectorXd currentVelocity = _knots[iknot].mVelocities;
    Eigen::VectorXd nextPosition = _knots[iknot + 1].mPositions;
    Eigen::VectorXd nextVelocity = _knots[iknot + 1].mVelocities;

    CubicSplineProblem problem(Eigen::Vector2d{0., segmentDuration}, 4, numDof);
    problem.addConstantConstraint(0, 0, zeroPosition);
    problem.addConstantConstraint(0, 1, currentVelocity);
    problem.addConstantConstraint(1, 0, nextPosition - currentPosition);
    problem.addConstantConstraint(1, 1, nextVelocity);
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
    const Eigen::Vector6d& _desiredTwist,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    const dart::dynamics::MetaSkeletonPtr _metaskeleton,
    const dart::dynamics::BodyNodePtr _bodyNode,
    double _optimizationTolerance,
    double _timestep,
    double _padding,
    Eigen::VectorXd& _jointVelocity)
{
  using dart::math::Jacobian;
  using dart::optimizer::Problem;
  using dart::optimizer::Solver;
  using Eigen::VectorXd;

  // Use LBFGS to find joint angles that won't violate the joint limits.
  const Jacobian jacobian = _metaskeleton->getWorldJacobian(_bodyNode);
  const auto properties = _stateSpace->getProperties();

  const std::size_t numDofs = properties.getNumDofs();
  const VectorXd positionLowerLimits = properties.getPositionLowerLimits();
  const VectorXd positionUpperLimits = properties.getPositionUpperLimits();
  VectorXd velocityLowerLimits = properties.getVelocityLowerLimits();
  VectorXd velocityUpperLimits = properties.getVelocityUpperLimits();

  auto currentState
      = _stateSpace->getScopedStateFromMetaSkeleton(_metaskeleton.get());
  VectorXd positions(numDofs);
  _stateSpace->convertStateToPositions(currentState, positions);
  VectorXd initialGuess = _metaskeleton->getVelocities();

  for (std::size_t i = 0; i < numDofs; ++i)
  {
    const double nextPositionLowerBound
        = positions[i] + _timestep * velocityLowerLimits[i];
    const double nextPositionUpperBound
        = positions[i] + _timestep * velocityUpperLimits[i];

    const double paddedPositionLowerLimit = positionLowerLimits[i] + _padding;
    const double paddedPositionUpperLimit = positionUpperLimits[i] - _padding;

    if (nextPositionLowerBound < paddedPositionLowerLimit)
    {
      const double feasibleVelocityLowerLimit
          = (paddedPositionLowerLimit - positions[i]) / _timestep;
      velocityLowerLimits[i]
          = std::max(velocityLowerLimits[i], feasibleVelocityLowerLimit);
    }

    if (nextPositionUpperBound > paddedPositionUpperLimit)
    {
      const double feasibleVelocityUpperLimit
          = (paddedPositionUpperLimit - positions[i]) / _timestep;
      velocityUpperLimits[i]
          = std::min(velocityUpperLimits[i], feasibleVelocityUpperLimit);
    }

    initialGuess[i] = common::clamp(
        initialGuess[i], velocityLowerLimits[i], velocityUpperLimits[i]);
  }

  const auto problem = std::make_shared<Problem>(numDofs);
  problem->setLowerBounds(velocityLowerLimits);
  problem->setUpperBounds(velocityUpperLimits);
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

} // namespace vectorfield
} // namespace planner
} // namespace aikido
