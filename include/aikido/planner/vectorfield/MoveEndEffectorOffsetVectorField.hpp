#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_

#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {


/// Vector field for moving end-effector by a direction and distance.
///
/// This class defines two callback functions for vectorfield planner.
/// One for generating joint velocity in MetaSkeleton state space,
/// and one for determining vectorfield planner status.
class MoveEndEffectorOffsetVectorField {
public:
  /// Constructor
  ///
  /// \param[in] _bn Body node of end-effector
  /// \param[in] _linearVelocity Linear velocity in all the directions
  /// \param[in] _startTime Start time of a planned trajectory
  /// \param[in] _endTime End time of a planned trajectory
  /// \param[in] _timestep Time step size
  /// \param[in] _linearGain Gain of P control on linear deviation
  /// \param[in] _linearTolerance Tolerance on linear deviation
  /// \param[in] _angularGain Gain of P control on angular deviation
  /// \param[in] _angularTolerance Tolerance on angular deviation
  /// \param[in] _optimizationTolerance Tolerance on optimization
  /// \param[in] _padding Padding to the boundary
  MoveEndEffectorOffsetVectorField(
    dart::dynamics::BodyNodePtr _bn,
    const Eigen::Vector3d& _linearVelocity,
    double _startTime,
    double _endTime,
    double _timestep,
    double _linearGain = 1.,
    double _linearTolerance = 0.01,
    double _angularGain = 1.,
    double _angularTolerance = 0.01,
    double _optimizationTolerance = 1e-3,
    double _padding = 1e-5
  );

  /// Vectorfield callback function
  ///
  /// \param[in] _stateSpace MetaSkeleton state space
  /// \param[in] _t Current time being planned
  /// \param[out] _dq Joint velocities
  /// \return Whether joint velocities are successfully computed
  bool operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& _stateSpace,
    double /* _t */,
    Eigen::VectorXd& _dq);

  /// Vectorfield planning status callback function
  ///
  /// \param[in] _stateSpace MetaSkeleton state space
  /// \param[in] _t Current time being planned
  /// \return Status of planning
  VectorFieldPlannerStatus operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& _stateSpace,
    double _t);

private:
  dart::dynamics::BodyNodePtr mBodynode;
  Eigen::Vector3d mVelocity;
  Eigen::Vector3d mLinearDirection;
  double mStartTime;
  double mEndTime;
  double mTimestep;
  double mLinearGain;
  double mLinearTolerance;
  double mAngularGain;
  double mAngularTolerance;
  double mOptimizationTolerance;
  double mPadding;
  Eigen::Isometry3d mStartPose;
  Eigen::Isometry3d mTargetPose;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEENDEFFECTOROFFSETVECTORFIELD_HPP_
