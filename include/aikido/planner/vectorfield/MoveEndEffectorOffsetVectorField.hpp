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
  /// Constructor
  ///
  /// \param[in] _stateSpace MetaSkeleton state space
  /// \param[in] _bn Body node of end-effector
  /// \param[in] _direction Unit vector in the direction of motion
  /// \param[in] _distance Minimum distance in meters
  /// \param[in] _maxDistance Maximum distance in meters
  /// \param[in] _positionTolerance Constraint tolerance in meters
  /// \param[in] _angularTolerance Constraint tolerance in radians
  /// \param[in] _initialStepSize Initial step size
  /// \param[in] _jointLimitTolerance If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0
  /// \param[in] _optimizationTolerance Tolerance on optimization
  MoveEndEffectorOffsetVectorField(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      dart::dynamics::BodyNodePtr _bn,
      const Eigen::Vector3d& _direction,
      double _distance,
      double _maxDistance = std::numeric_limits<double>::max(),
      double _positionTolerance = 0.01,
      double _angularTolerance = 0.15,
      double _initialStepSize = 1e-1,
      double _jointLimitTolerance = 3e-2,
      double _optimizationTolerance = 1e-3);

  /// Vectorfield callback function
  ///
  /// \param[in] _q Position in configuration space
  /// \param[in] _t Current time being planned
  /// \param[out] _qd Joint velocities
  /// \return Whether joint velocities are successfully computed
  virtual bool getJointVelocities(
      const Eigen::VectorXd& _q, double _t, Eigen::VectorXd& _qd) override;

  /// Vectorfield planning status callback function
  ///
  /// \param[in] _q Position in configuration space
  /// \param[in] _t Current time being planned
  /// \return Status of planning
  virtual VectorFieldPlannerStatus checkPlanningStatus(
      const Eigen::VectorXd& _q, double _t) override;

protected:
  Eigen::Vector3d mDirection;
  double mDistance;
  double mMaxDistance;
  double mPositionTolerance;
  double mAngularTolerance;
  double mInitialStepSize;
  double mJointLimitTolerance;
  double mOptimizationTolerance;
  Eigen::Isometry3d mStartPose;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef
       // AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_
