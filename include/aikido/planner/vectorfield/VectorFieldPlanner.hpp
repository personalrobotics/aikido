#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_

#include <functional>
#include <dart/common/Timer.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/vectorfield/ConfigurationSpaceVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerExceptions.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// VectorField Planner generates a trajectory by following a vector field
/// defined in joint space.
/// A planned trajectroy ends either by a predefined termination criterion or a
/// integral time.
///
/// This class defines two callback functions for a integrator.
/// step() provides joint velocities the vector field planner should follow
/// check() is called after each integration step.
class VectorFieldPlanner
{
public:
  /// Constructor
  ///
  /// \param[in] _vectorField Vector field in configuration space
  /// \param[in] _constraint Constraint to be satisfied

  VectorFieldPlanner(
      const aikido::planner::vectorfield::ConfigurationSpaceVectorFieldPtr
          _vectorField,
      const aikido::constraint::TestablePtr _constraint,
      double _initialStepSize = 0.1);

  /// Vectorfield callback function
  ///
  /// \param[in] _q Position in configuration space
  /// \param[out] _qd Joint velocities
  /// \param[in] _t Current time being planned
  void step(const Eigen::VectorXd& _q, Eigen::VectorXd& _qd, double _t);

  /// Check status after every intergration step
  ///
  /// \param[in] _q Position in configuration space
  /// \param[in] _t Current time being planned
  void check(const Eigen::VectorXd& _q, double _t);

  /// Generate a trajectory following the vector field along given time
  ///
  /// \param[in] _integrationTimeInterval Position in configuration space
  /// \param[in] _timelimit Timelimit for integration calculation
  /// \param[in] _useCollisionChecking Whether collision checking is
  /// considered in planning
  /// \param[in] _useDofLimitChecking Whether Dof Limits are considered
  /// in planning
  /// \return A trajectory following the vector field
  std::unique_ptr<aikido::trajectory::Spline> followVectorField(
      double _integrationTimeInterval,
      double _timelimit,
      double _useCollisionChecking = true,
      double _useDofLimitChecking = true);

protected:
  aikido::planner::vectorfield::ConfigurationSpaceVectorFieldPtr mVectorField;
  aikido::constraint::TestablePtr mConstraint;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  dart::dynamics::BodyNodePtr mBodyNode;
  dart::common::Timer mTimer;
  double mTimelimit;

  std::vector<Knot> mKnots;

  double mInitialStepSize;
  int mCacheIndex;
  int mIndex;

  bool mEnableCollisionCheck;
  bool mEnableDofLimitCheck;
};

/// Plan to a trajectory that moves the end-effector by a given direction and
/// distance.
///
/// \param[in] _stateSpace MetaSkeleton state space
/// \param[in] _bn Body node of the end-effector
/// \param[in] _constraint Trajectory-wide constraint that must be satisfied
/// \param[in] _direction Direction of moving the end-effector
/// \param[in] _distance  Distance of moving the end-effector
/// \param[in] _maxDistance Max distance of moving the end-effector
/// \param[in] _positionTolerance How a planned trajectory is allowed to
/// deviated from a straight line segment defined by the direction and the
/// distance
/// \param[in] angularTolerance How a planned trajectory is allowed to deviate
/// from a given direction
/// \param[in] _useCollisionChecking Whether collision checking is
/// considered in planning
/// \param[in] _useDofLimitChecking Whether Dof Limits are considered
/// in planning
/// \param[in] _initialStepSize Initial step size
/// \param[in] _jointLimitTolerance If less then this distance to joint
/// limit, velocity is bounded in that direction to 0
/// \param[in] _optimizationTolerance Tolerance on optimization
/// \param[in] _timelimit timeout in seconds
/// \param[in] _integralTimeInterval The time interval to integrate over
/// \return Trajectory or \c nullptr if planning failed
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& _stateSpace,
    const dart::dynamics::BodyNodePtr& _bn,
    const aikido::constraint::TestablePtr& _constraint,
    const Eigen::Vector3d& _direction,
    double _distance,
    double _maxDistance = std::numeric_limits<double>::max(),
    double _positionTolerance = 0.01,
    double _angularTolerance = 0.15,
    double _useCollisionChecking = true,
    double _useDofLimitChecking = true,
    double _initialStepSize = 1e-2,
    double _jointLimitTolerance = 3e-2,
    double _optimizationTolerance = 1e-3,
    double _timelimit = 5.0,
    double _integralTimeInterval = 10.0);

/// Plan to an end effector pose by following a geodesic loss function
/// in SE(3) via an optimized Jacobian.

///
/// \param[in] _stateSpace MetaSkeleton state space
/// \param[in] _bn Body node of the end-effector
/// \param[in] _constraint Trajectory-wide constraint that must be satisfied
/// \param[in] _goalPose Desired end-effector pose
/// \param[in] _positionErrorTolerance How a planned trajectory is allowed to
/// deviated from a straight line segment defined by the direction and the
/// distance
/// \param[in] _useCollisionChecking Whether collision checking is
/// considered in planning
/// \param[in] _useDofLimitChecking Whether Dof Limits are considered
/// in planning
/// \param[in] _initialStepSize Initial step size
/// \param[in] _jointLimitTolerance If less then this distance to joint
/// limit, velocity is bounded in that direction to 0
/// \param[in] _optimizationTolerance Tolerance on optimization
/// \param[in] _timelimit timeout in seconds
/// \param[in] _integralTimeInterval The time interval to integrate over
/// \return Trajectory or \c nullptr if planning failed
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorPose(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& _stateSpace,
    const dart::dynamics::BodyNodePtr& _bn,
    const aikido::constraint::TestablePtr& _constraint,
    const Eigen::Isometry3d& _goalPose,
    double _poseErrorTolerance,
    double _useCollisionChecking = true,
    double _useDofLimitChecking = true,
    double _initialStepSize = 1e-2,
    double _jointLimitTolerance = 3e-2,
    double _optimizationTolerance = 1.,
    double _timelimit = 5.0,
    double _integralTimeInterval = 10.0);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
