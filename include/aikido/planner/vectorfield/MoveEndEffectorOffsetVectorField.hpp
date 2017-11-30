#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/ConfigurationSpaceVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Vector field for moving end-effector by a direction and distance.
///
/// This class defines two callback functions for vectorfield planner.
/// One for generating joint velocity in MetaSkeleton state space,
/// and one for determining vectorfield planner status.
class MoveEndEffectorOffsetVectorField : public ConfigurationSpaceVectorField
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
  /// \param[in] distance Minimum distance in meters
  /// \param[in] maxDistance Maximum distance in meters
  /// \param[in] positionTolerance Constraint tolerance in meters
  /// \param[in] angularTolerance Constraint tolerance in radians
  /// \param[in] linearVelocityGain Linear velocity gain in workspace.
  /// \param[in] initialStepSize Initial step size.
  /// \param[in] jointLimitPadding If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0.
  /// \param[in] optimizationTolerance Tolerance on optimization.
  MoveEndEffectorOffsetVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bn,
      const Eigen::Vector3d& direction,
      double distance,
      double maxDistance = InvalidMaxDistance,
      double positionTolerance = 0.01,
      double angularTolerance = 0.15,
      double linearVelocityGain = 1.0,
      double initialStepSize = 1e-1,
      double jointLimitPadding = 3e-2,
      double optimizationTolerance = 1e-3);

  /// Vectorfield callback function.
  ///
  /// \param[out] qd Joint velocities.
  /// \return Whether joint velocities are successfully computed.
  bool getJointVelocities(Eigen::VectorXd& qd) const override;

  /// Vectorfield planning status callback function.
  ///
  /// \return Status of planning.
  VectorFieldPlannerStatus checkPlanningStatus() const override;

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

  /// Initial step size in adaptive integration.
  double mInitialStepSize;

  /// Padding for joint limits.
  double mJointLimitPadding;

  /// Tolerance of optimization solver.
  double mOptimizationTolerance;

  /// Start pose of the end-effector.
  Eigen::Isometry3d mStartPose;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
       // AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_
