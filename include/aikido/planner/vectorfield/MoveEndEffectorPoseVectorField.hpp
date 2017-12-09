#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/BodyNodePoseVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Vector field for moving end-effector to a goal pose in SE(3).
/// It defines a vector field in meta-skeleton state space that moves
/// an end-effector a desired pose in SE(3) by following a geodesic
/// loss function in SE(3) via an optimized Jacobian.
/// The geodesic loss function is defined as the geodesic (shortest
///  path) from the current pose to the goal pose.
///
/// This class defines two callback functions for vectorfield planner.
/// One for generating joint velocity in MetaSkeleton state space,
/// and one for determining vectorfield planner status.
class MoveEndEffectorPoseVectorField : public BodyNodePoseVectorField
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace MetaSkeleton state space.
  /// \param[in] bn Body node of end-effector.
  /// \param[in] goalPose Desired end-effector pose.
  /// \param[in] poseErrorTolerance Constraint error tolerance in meters.
  /// \param[in] r Conversion of radius to meters in computing Geodesic
  /// distance.
  /// \param[in] linearVelocityGain Linear velocity gain in workspace.
  /// \param[in] angularVelocityGain Angular velocity gain in workspace.
  /// \param[in] initialStepSize Initial step size.
  /// \param[in] jointLimitPadding If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0.
  MoveEndEffectorPoseVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bn,
      const Eigen::Isometry3d& goalPose,
      double poseErrorTolerance,
      double r,
      double linearVelocityGain,
      double angularVelocityGain,
      double initialStepSize,
      double jointLimitPadding);

  // Documentation inherited.
  bool evaluateCartesianVelocity(
      const Eigen::Isometry3d& pose,
      Eigen::Vector6d& cartesianVelocity) const override;

  // Documentation inherited.
  VectorFieldPlannerStatus evaluateCartesianStatus(
      const Eigen::Isometry3d& pose) const override;

protected:
  /// Goal pose.
  Eigen::Isometry3d mGoalPose;

  /// Tolerance of pose error.
  double mPoseErrorTolerance;

  /// Conversion ratio from radius to meter.
  double mConversionRatioFromRadiusToMeter;

  /// Linear velocity gain.
  double mLinearVelocityGain;

  /// Angular velocit gain.
  double mAngularVelocityGain;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
       // AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_
