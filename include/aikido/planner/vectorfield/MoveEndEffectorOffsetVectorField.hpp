#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/BodyNodePoseVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Vector field for moving end-effector by a direction and distance.
/// It defines a vector field in meta-skeleton state space that moves
/// an end-effector a desired offset with move-hand-straight constraint.
/// The move-hand-straight constraint is defined by the direction and
/// distance range [minimum distance, maximum distance).
/// Movement less than minimum distance will return failure.
/// The motion will not move further than maximum distance.
///
/// This class defines two callback functions for vectorfield planner.
/// One for generating joint velocity in MetaSkeleton state space,
/// and one for determining vectorfield planner status.
class MoveEndEffectorOffsetVectorField : public BodyNodePoseVectorField
{
public:
  static double InvalidMaxDistance;

  /// Constructor
  ///
  /// (1) When maxDistance is set to InvalidMaxDistance, planning
  /// will terminate and return a path if moved distance is larger
  /// than distance.
  /// (2) When maxDistance is not set to InvalidMaxDistance,
  /// planning will terminate and return a path, the path length
  /// of which is [distance, maxDistance].
  ///
  /// \param[in] stateSpace MetaSkeleton state space
  /// \param[in] bn Body node of end-effector
  /// \param[in] direction Unit vector in the direction of motion
  /// \param[in] minDistance Minimum distance in meters
  /// \param[in] maxDistance Maximum distance in meters
  /// \param[in] positionTolerance Constraint tolerance in meters
  /// \param[in] angularTolerance Constraint tolerance in radians
  /// \param[in] linearVelocityGain Linear velocity gain in workspace.
  /// \param[in] initialStepSize Initial step size.
  /// \param[in] jointLimitPadding If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0.
  MoveEndEffectorOffsetVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bn,
      const Eigen::Vector3d& direction,
      double minDistance,
      double maxDistance = InvalidMaxDistance,
      double positionTolerance = 0.01,
      double angularTolerance = 0.15,
      double linearVelocityGain = 1.0,
      double initialStepSize = 5e-2,
      double jointLimitPadding = 3e-2);

  // Documentation inherited.
  bool evaluateVelocity(
      const aikido::statespace::StateSpace::State* state,
      Eigen::VectorXd& qd) const override;

  // Documentation inherited.
  VectorFieldPlannerStatus evaluateStatus(
      const aikido::statespace::StateSpace::State* state) const override;

protected:
  /// Movement direction.
  Eigen::Vector3d mDirection;

  /// Movement distance.
  double mDistance;

  /// Maximum distance allowed to move.
  double mMaxDistance;

  /// Tolerance of linear deviation error.
  double mPositionTolerance;

  /// Tolerance of angular error.
  double mAngularTolerance;

  /// Linear velocity gain.
  double mLinearVelocityGain;

  /// Initial step size of integrator.
  double mInitialStepSize;

  /// Padding for joint limits.
  double mJointLimitPadding;

  /// Start pose of the end-effector.
  Eigen::Isometry3d mStartPose;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
       // AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_
