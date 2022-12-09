#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_

#include <limits>

#include "aikido/planner/vectorfield/BodyNodePoseVectorField.hpp"

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
  /// \param[in] metaskeleton MetaSkeleton to plan with
  /// \param[in] bn Body node of end-effector.
  /// \param[in] goalPose Desired end-effector pose.
  /// \param[in] goalTolerance Geodesic Distance from goal to stop planning.
  /// \param[in] r Conversion of radius to meters in computing goalTolerance.
  /// \param[in] positionTolerance Straight-line constraint tolerance in meters.
  /// \param[in] angularTolerance Rotation deviation tolerance in radians.
  /// \param[in] maxStepSize The maximum step size used to guarantee
  /// that the integrator does not step out of joint limits.
  /// \param[in] jointLimitPadding If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0.
  MoveEndEffectorPoseVectorField(
      aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaskeleton,
      ::dart::dynamics::ConstBodyNodePtr bn,
      const Eigen::Isometry3d& goalPose,
      double goalTolerance,
      double r,
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
  /// Goal pose.
  Eigen::Isometry3d mGoalPose;

  /// Tolerance of pose error.
  double mGoalTolerance;

  /// Conversion ratio from radius to meter.
  double mConversionRatioFromRadianToMeter;

  /// Tolerance of linear deviation error.
  double mPositionTolerance;

  /// Tolerance of angular error.
  double mAngularTolerance;

  /// Start pose of the end-effector.
  Eigen::Isometry3d mStartPose;

  /// Cache for desired twist between start and goal.
  Eigen::Vector6d mDesiredTwist;
  Eigen::Vector3d mDirection;
  Eigen::Vector3d mRotation;

  /// Cache of last pose error
  std::shared_ptr<double> mLastPoseError;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
       // AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTORPOSEVECTORFIELD_HPP_
