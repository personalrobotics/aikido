#include <aikido/trajectory/Spline.hpp>
#include "VectorFieldUtil.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

std::unique_ptr<aikido::trajectory::Spline>
convertToSpline(const std::vector<Knot>& knots, ptrdiff_t cache_index,
                aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace)
{
  using dart::common::make_unique;

  std::size_t num_dof = stateSpace->getMetaSkeleton()->getNumDofs();
  // Construct the output spline.
  Eigen::VectorXd times(cache_index);
  std::transform(
    knots.begin(),
    knots.begin() + cache_index,
    times.data(),
    [](Knot const& knot) { return knot.t; });

  auto _outputTrajectory
    = make_unique<aikido::trajectory::Spline>(stateSpace);

  using CubicSplineProblem = aikido::common::
    SplineProblem<double, int, 4, Eigen::Dynamic, Eigen::Dynamic>;

  const Eigen::VectorXd zeroPosition = Eigen::VectorXd::Zero(num_dof);
  auto currState = stateSpace->createState();
  for (int iknot = 0; iknot < cache_index - 1; ++iknot)
  {
    const double segmentDuration = knots[iknot + 1].t - knots[iknot].t;
    Eigen::VectorXd currentPosition = knots[iknot].values.row(0);
    Eigen::VectorXd currentVelocity = knots[iknot].values.row(1);
    Eigen::VectorXd nextPosition = knots[iknot + 1].values.row(0);
    Eigen::VectorXd nextVelocity = knots[iknot + 1].values.row(1);

    CubicSplineProblem problem(
      Eigen::Vector2d{0., segmentDuration}, 4, num_dof);
    problem.addConstantConstraint(0, 0, zeroPosition);
    problem.addConstantConstraint(0, 1, currentVelocity);
    problem.addConstantConstraint(1, 0, nextPosition - currentPosition);
    problem.addConstantConstraint(1, 1, nextVelocity);
    const auto solution = problem.fit();
    const auto coefficients = solution.getCoefficients().front();

    // Build the output trajectory/
    stateSpace->expMap(currentPosition, currState);
    _outputTrajectory->addSegment(coefficients, segmentDuration, currState);
  }
  return _outputTrajectory;
}

DesiredTwistFunction::DesiredTwistFunction(const Twist& _twist, const Jacobian& _jacobian)
  : dart::optimizer::Function("DesiredTwistFunction")
  , mTwist(_twist)
  , mJacobian(_jacobian)
{
}

double DesiredTwistFunction::eval(const Eigen::VectorXd& _qd)
{
  return 0.5 * (mJacobian * _qd - mTwist).squaredNorm();
}

void DesiredTwistFunction::evalGradient(
  const Eigen::VectorXd& _qd, Eigen::Map<Eigen::VectorXd> _grad)
{
  _grad = mJacobian.transpose() * (mJacobian * _qd - mTwist);
}

bool ComputeJointVelocityFromTwist(const Eigen::Vector6d& _desiredTwist,
                                   const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
                                   const dart::dynamics::BodyNodePtr _bodyNode,
                                   const double _optimizationTolerance,
                                   const double _timestep,
                                   const double _padding,
                                   Eigen::VectorXd* _jointVelocity)
{
  using dart::math::Jacobian;
  using dart::optimizer::Problem;
  using dart::optimizer::Solver;
  using Eigen::VectorXd;
  // Use LBFGS to find joint angles that won't violate the joint limits.
  const Jacobian jacobian
      = _stateSpace->getMetaSkeleton()->getWorldJacobian(_bodyNode);

  const size_t numDofs = _stateSpace->getMetaSkeleton()->getNumDofs();
  VectorXd lowerLimits(numDofs);
  VectorXd upperLimits(numDofs);

  VectorXd positions = _stateSpace->getMetaSkeleton()->getPositions();
  VectorXd positionLowerLimits
      = _stateSpace->getMetaSkeleton()->getPositionLowerLimits();
  VectorXd positionUpperLimits
      = _stateSpace->getMetaSkeleton()->getPositionUpperLimits();
  VectorXd velocityLowerLimits
      = _stateSpace->getMetaSkeleton()->getVelocityLowerLimits();
  VectorXd velocityUpperLimits
      = _stateSpace->getMetaSkeleton()->getVelocityUpperLimits();

  for (size_t i = 0; i < numDofs; ++i)
  {
    const double position = positions[i];
    const double positionLowerLimit = positionLowerLimits[i];
    const double positionUpperLimit = positionUpperLimits[i];
    const double velocityLowerLimit = velocityLowerLimits[i];
    const double velocityUpperLimit = velocityUpperLimits[i];

    if (position
        < positionLowerLimit - _timestep * velocityLowerLimit + _padding)
      lowerLimits[i] = 0;
    else
      lowerLimits[i] = velocityLowerLimit;

    if (position
        > positionUpperLimit - _timestep * velocityUpperLimit - _padding)
      upperLimits[i] = 0;
    else
      upperLimits[i] = velocityUpperLimit;
  }

  const auto problem = std::make_shared<Problem>(numDofs);
  problem->setLowerBounds(lowerLimits);
  problem->setUpperBounds(upperLimits);
  problem->setObjective(
      std::make_shared<DesiredTwistFunction>(_desiredTwist, jacobian));

  dart::optimizer::NloptSolver solver(problem, nlopt::LD_LBFGS);
  if (!solver.solve())
  {
    return false;
  }
  double optimalVal = problem->getOptimumValue();
  if(optimalVal > _optimizationTolerance)
  {
    return false;
  }

  *_jointVelocity = problem->getOptimalSolution();
  return true;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
