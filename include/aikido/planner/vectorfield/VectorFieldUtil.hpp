#ifndef AIKIDO_PLANNER_VECTORFIELD_UTIL_H_
#define AIKIDO_PLANNER_VECTORFIELD_UTIL_H_

#include <dart/dynamics/BodyNode.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Solver.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>
#include <aikido/common/Spline.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

struct Knot
{
  double t;
  Eigen::Matrix<double, 2, Eigen::Dynamic> values;
};

std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& knots,
    ptrdiff_t cache_index,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace);

class DesiredTwistFunction : public dart::optimizer::Function
{
public:
  using Twist = Eigen::Matrix<double, 6, 1>;
  using Jacobian = dart::math::Jacobian;

  DesiredTwistFunction(const Twist& _twist, const Jacobian& _jacobian);
  double eval(const Eigen::VectorXd& _qd) override;
  void evalGradient(
      const Eigen::VectorXd& _qd, Eigen::Map<Eigen::VectorXd> _grad) override;

private:
  Twist mTwist;
  Jacobian mJacobian;
};

bool ComputeJointVelocityFromTwist(
    const Eigen::Vector6d& _desiredTwist,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    const dart::dynamics::BodyNodePtr _bodyNode,
    const double _optimizationTolerance,
    const double _timestep,
    const double _padding,
    Eigen::VectorXd* _jointVelocity);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_UTIL_H_
