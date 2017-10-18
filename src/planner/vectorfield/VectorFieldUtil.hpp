#ifndef AIKIDO_PLANNER_VECTORFIELD_UTIL_H_
#define AIKIDO_PLANNER_VECTORFIELD_UTIL_H_

#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Solver.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

class DesiredTwistFunction : public dart::optimizer::Function
{
public:
  using Twist = Eigen::Matrix<double, 6, 1>;
  using Jacobian = dart::math::Jacobian;

  DesiredTwistFunction(const Twist& _twist, const Jacobian& _jacobian)
    : dart::optimizer::Function("DesiredTwistFunction")
    , mTwist(_twist)
    , mJacobian(_jacobian)
  {
  }

  double eval(const Eigen::VectorXd& _qd) override
  {
    return 0.5 * (mJacobian * _qd - mTwist).squaredNorm();
  }

  void evalGradient(
      const Eigen::VectorXd& _qd, Eigen::Map<Eigen::VectorXd> _grad) override
  {
    _grad = mJacobian.transpose() * (mJacobian * _qd - mTwist);
  }

private:
  Twist mTwist;
  Jacobian mJacobian;
};

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

#endif // AIKIDO_PLANNER_VECTORFIELD_UTIL_H_
