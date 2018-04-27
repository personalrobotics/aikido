#ifndef AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_
#define AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a desired end-effector pose.
class ConfigurationToEndEffectorPose : public Problem
{
public:
  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param endEffectorBodyNode BodyNode to be planned to move to a desired
  /// pose.
  /// \param startState Start state.
  /// \param goalPose Goal pose.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToEndEffectorPose(
      statespace::StateSpacePtr stateSpace,
      dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      const statespace::StateSpace::State* startState,
      const Eigen::Isometry3d& goalPose,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the end-effector BodyNode to be planned to move to a desired pose.
  dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Returns the goal pose.
  const Eigen::Isometry3d& getGoalPose() const;

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// End-effector body node.
  dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;

  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal pose.
  Eigen::Isometry3d mGoalPose;

  /// Trajectory-wide constraint that must be satisfied.
  constraint::ConstTestablePtr mConstraint;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_
