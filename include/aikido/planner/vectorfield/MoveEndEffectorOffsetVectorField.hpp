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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// Constructor
  ///
  /// \param[in] stateSpace MetaSkeleton state space.
  /// \param[in] metaskeleton MetaSkeleton to plan with
  /// \param[in] bn Body node of end-effector.
  /// \param[in] direction Unit vector in the direction of motion.
  /// \param[in] minDistance Minimum distance in meters.
  /// \param[in] maxDistance Maximum distance in meters. Maximum distance
  /// should be larger than minimum distance so that a feasible solution
  /// could be found.
  /// \param[in] positionTolerance Constraint tolerance in meters.
  /// \param[in] angularTolerance Constraint tolerance in radians.
  /// \param[in] maxStepSize The maximum step size used to guarantee
  /// that the integrator does not step out of joint limits.
  /// \param[in] jointLimitPadding If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0.
  MoveEndEffectorOffsetVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      dart::dynamics::MetaSkeletonPtr metaskeleton,
      dart::dynamics::BodyNodePtr bn,
      const Eigen::Vector3d& direction,
      double minDistance,
      double maxDistance,
      double positionTolerance,
      double angularTolerance,
      double maxStepSize,
      double jointLimitPadding);

  // Documentation inherited.
  bool evaluateCartesianVelocity(
      const Eigen::Isometry3d& pose,
      Eigen::Vector6d& cartesianVelocity) const override;

  // Documentation inherited.
  VectorFieldPlannerStatus evaluateCartesianStatus(
      const Eigen::Isometry3d& pose) const override;

protected:
  /// Movement direction.
  Eigen::Vector3d mDirection;

  /// Minimum movement distance.
  double mMinDistance;

  /// Maximum distance allowed to move.
  double mMaxDistance;

  /// Tolerance of linear deviation error.
  double mPositionTolerance;

  /// Tolerance of angular error.
  double mAngularTolerance;

  /// Start pose of the end-effector.
  Eigen::Isometry3d mStartPose;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
       // AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_
