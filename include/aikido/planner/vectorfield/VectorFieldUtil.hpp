#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_

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
  double mT;
  Eigen::VectorXd mPositions;
  Eigen::VectorXd mVelocities;
};

/// Convert a sequence of knots into a Spline trajectory.
///
/// \param[in] _knots A sequence of knots
/// \param[in] _cache_index Total cache index number
/// \param[in] _stateSpace MetaSkeleton state space
/// \return A Spline trajectory
std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const std::vector<Knot>& _knots,
    int _cacheIndex,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace);

/// A function class that defines an objective. The objective measures
/// the difference between a desired twist and Jacobian * joint velocities.
class DesiredTwistFunction : public dart::optimizer::Function
{
public:
  using Twist = Eigen::Vector6d;
  using Jacobian = dart::math::Jacobian;

  /// Constructor.
  ///
  /// \param[in] twist A desired twist
  /// \param[in] jacobian System Jacobian
  DesiredTwistFunction(const Twist& twist, const Jacobian& jacobian);

  /// Implementation inherited.
  /// Evaluating an objective by a state value.
  ///
  /// \param[in] _qd Joint velocities
  /// \return Objective value
  double eval(const Eigen::VectorXd& _qd) override;

  /// Implementation inherited.
  /// Evaluating gradient of an objective by a state value.
  /// \param[in] _qd Joint velocities
  /// \param[out] _grad Gradient of a defined objective
  void evalGradient(
      const Eigen::VectorXd& _qd, Eigen::Map<Eigen::VectorXd> _grad) override;

private:
  Twist mTwist;
  Jacobian mJacobian;
};

/// Compute joint velocity from a given twist.
///
/// \param[in] _desiredTwist Desired twist, which consists of angular velocity
/// and linear velocity
/// \param[in] _stateSpace MetaSkeleton state space
/// \param[in]  _bodyNode Body node of the end-effector
/// \param[in] _optimizationTolerance Callback of vector field calculation
/// \param[in] _timestep How long will the computed joint velocities be executed
/// \param[in] _padding Padding for joint limits
/// \param[out] _jointVelocity Calculated joint velocities
bool computeJointVelocityFromTwist(
    const Eigen::Vector6d& _desiredTwist,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    const dart::dynamics::BodyNodePtr _bodyNode,
    double _optimizationTolerance,
    double _timestep,
    double _padding,
    Eigen::VectorXd& _jointVelocity);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDUTIL_HPP_
