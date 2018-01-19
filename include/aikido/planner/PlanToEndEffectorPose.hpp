#ifndef AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_
#define AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/smart_pointer.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/smart_pointer.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a given end effector pose.
///
/// Plan a trajectory from start state to goal state by using an interpolator to
/// interpolate between them.
class PlanToEndEffectorPose : public Problem
{
public:
  using ReturnTrajectoryType = trajectory::Interpolated;

  class Result;

  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param bodyNode Body Node or Robot for which the path is to be planned.
  /// \param startState Start state.
  /// \param goalPose Goal pose.
  /// \param interpolator Interpolator used to produce the output trajectory.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible to \c constraint's state space.
  PlanToEndEffectorPose(
      statespace::ConstStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bodyNode,
      const statespace::StateSpace::State* startState,
      const Eigen::Isometry3d& goalPose,
      statespace::InterpolatorPtr interpolator,
      constraint::TestablePtr constraint);

  const std::string& getName() const override;

  static const std::string& getStaticName();

  dart::dynamics::BodyNodePtr getBodyNode();

  const statespace::StateSpace::State* getStartState() const;

  const Eigen::Isometry3d& getGoalPose() const;

  statespace::InterpolatorPtr getInterpolator();

  statespace::ConstInterpolatorPtr getInterpolator() const;

  constraint::TestablePtr getConstraint();

  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// Body Node or Robot.
  dart::dynamics::BodyNodePtr mBodyNode;

  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal pose.
  const Eigen::Isometry3d mGoalPose;

  /// Interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr mInterpolator;

  /// Trajectory-wide constraint that must be satisfied.
  constraint::TestablePtr mConstraint;
};

class PlanToEndEffectorPose::Result : public Problem::Result
{
public:
protected:
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_
