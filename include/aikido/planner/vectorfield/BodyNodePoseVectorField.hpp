#ifndef AIKIDO_PLANNER_VECTORFIELD_BODYNODEPOSEVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_BODYNODEPOSEVECTORFIELD_HPP_

#include <dart/dynamics/BodyNode.hpp>
#include <aikido/planner/vectorfield/VectorField.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

AIKIDO_DECLARE_POINTERS(BodyNodePoseVectorField)

/// This class is a vector field over a MetaSkeletonStateSpace that is
///  defined in terms of a Cartesian vector field over the pose of a
/// BodyNode in that MetaSkeleton.
/// The Jacobian pseudoinverse is used to map velocities in se(3) to
/// velocities in configuration space.
/// If multiple solutions exist, i.e. the kinematic chain leading to
/// the BodyNode in the MetaSkeleton is redundant, then a heuristic is
/// used to select velocities from the null space to avoid violating
/// joint limits.
class BodyNodePoseVectorField : public VectorField
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace MetaSkeleton state space.
  /// \param[in] metaskeleton MetaSkeleton to plan with
  /// \param[in] bn Body node of end-effector.
  /// \param[in] maxStepSize The maximum step size used to guarantee
  /// that the integrator does not step out of joint limits.
  /// \param[in] jointLimitpadding If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0.
  /// \param[in] enforceJointVelocityLimits Whether joint velocity limits
  /// are considered in computation.
  BodyNodePoseVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr
          metaSkeletonStateSpace,
      dart::dynamics::MetaSkeletonPtr metaSkeleton,
      dart::dynamics::BodyNodePtr bodyNode,
      double maxStepSize,
      double jointLimitPadding,
      bool enforceJointVelocityLimits = false);

  // Documentation inherited.
  bool evaluateVelocity(
      const aikido::statespace::StateSpace::State* state,
      Eigen::VectorXd& qd) const override;

  // Documentation inherited.
  VectorFieldPlannerStatus evaluateStatus(
      const aikido::statespace::StateSpace::State* state) const override;

  /// Evaluate Cartesian velocity by current pose of body node.
  ///
  /// \param[in] pose Current pose of body node.
  /// \param[out] cartesianVelocity Cartesian velocity defined in se(3).
  virtual bool evaluateCartesianVelocity(
      const Eigen::Isometry3d& pose,
      Eigen::Vector6d& cartesianVelocity) const = 0;

  /// Evaluate current pose to determine the status of planning.
  ///
  /// \param[in] pose Current pose of body node.
  /// \return Status of planner.
  virtual VectorFieldPlannerStatus evaluateCartesianStatus(
      const Eigen::Isometry3d& pose) const = 0;

  // Documentation inherited.
  bool evaluateTrajectory(
      const aikido::trajectory::Trajectory& trajectory,
      const aikido::constraint::Testable* constraint,
      double evalStepSize,
      double& evalTimePivot,
      bool excludeEndTime) const override;

  /// Return meta skeleton state space.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr
  getMetaSkeletonStateSpace();

  /// Return const meta skeleton state space.
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
  getMetaSkeletonStateSpace() const;

  /// Returns meta skeleton.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton();

  /// Returns const meta skeleton.
  dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const;

  /// Returns body node of end-effector.
  dart::dynamics::BodyNodePtr getBodyNode();

  /// Returns const body node of end-effector.
  dart::dynamics::ConstBodyNodePtr getBodyNode() const;

protected:
  /// Meta skeleton state space.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// Meta Skeleton
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  /// BodyNode
  dart::dynamics::BodyNodePtr mBodyNode;

  /// Maximum step size of integrator.
  double mMaxStepSize;

  /// Padding of joint limits.
  double mJointLimitPadding;

  /// Joint velocities lower limits.
  Eigen::VectorXd mVelocityLowerLimits;

  /// Joint velocities upper limits.
  Eigen::VectorXd mVelocityUpperLimits;

  /// Enfoce joint velocity limits
  bool mEnforceJointVelocityLimits;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_BODYNODEPOSEVECTORFIELD_HPP_
