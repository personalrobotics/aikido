#ifndef AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_
#define AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a given end effector pose.
///
/// Plan a trajectory from start state to goal state by using an interpolator to
/// interpolate between them.
class ConfigurationToEndEffectorPose : public Problem
{
public:
  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param bodyNode Body Node or Robot for which the path is to be planned.
  /// \param startState Start state.
  /// \param goalPose Goal pose.
  /// \param interpolator Interpolator used to produce the output trajectory.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToEndEffectorPose(
      statespace::StateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bodyNode,
      const statespace::StateSpace::State* startState,
      const Eigen::Isometry3d& goalPose,
      statespace::InterpolatorPtr interpolator,
      constraint::TestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the name of the planner problem.
  static const std::string& getStaticType();

  /// Returns the body node or robot for which the path is to be planned.
  dart::dynamics::BodyNodePtr getBodyNode();

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Returns the goal pose.
  const Eigen::Isometry3d& getGoalPose() const;

  /// Returns the interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr getInterpolator();

  /// Returns the interpolator used to produce the output trajectory.
  statespace::ConstInterpolatorPtr getInterpolator() const;

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::TestablePtr getConstraint();

  /// Returns the constraint that must be satisfied throughout the trajectory.
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

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_
