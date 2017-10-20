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

/// convert a sequence of knots into a Spline trajectory.
/// \param knots a sequence of knots
/// \param cache_index total cache index number
/// \param stateSpace MetaSkeleton state space
/// \return A Spline trajectory
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& knots,
    ptrdiff_t cache_index,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace);

/// A function class that defines an objective. The objective measures
/// the difference between a desired twist and Jacobian * joint velocities.
class DesiredTwistFunction : public dart::optimizer::Function
{
public:
  using Twist = Eigen::Matrix<double, 6, 1>;
  using Jacobian = dart::math::Jacobian;

  /// Constructor.
  /// \param _twist a desired twist
  /// \param _jacobian system Jacobian
  DesiredTwistFunction(const Twist& _twist, const Jacobian& _jacobian);
  /// Evluating an objective by a state vaue
  /// \param joinit velocities
  /// \return objective value
  double eval(const Eigen::VectorXd& _qd) override;
  ///
  void evalGradient(
      const Eigen::VectorXd& _qd, Eigen::Map<Eigen::VectorXd> _grad) override;

private:
  Twist mTwist;
  Jacobian mJacobian;
};

/// Compute joint velocity from a given twist.
/// \param _desiredTwist desired twist, which consists of angular velocity and
/// linear velocity
/// \param _stateSpace MetaSkeleton state space
/// \param _bodyNode body node of the end-effector
/// \param _optimizationTolerance callback of vector field calculation
/// \param _timestep how long will the computed joint velocities be executed
/// \param _padding padding for joint limits
/// \param[out] _jointVelocity calculated joint velocities
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
