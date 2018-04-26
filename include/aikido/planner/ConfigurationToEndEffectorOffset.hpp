#ifndef AIKIDO_PLANNER_PLANTOENDEFFECTOROFFSET_HPP_
#define AIKIDO_PLANNER_PLANTOENDEFFECTOROFFSET_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to desired end-effector offset while maintaining
/// the current end-effector orientation.
class ConfigurationToEndEffectorOffset : public Problem
{
public:
  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param bodyNode Body Node or robot for which the path is to be planned.
  /// \param startState Start state.
  /// \param direction Direction of motion [unit vector in the world frame].
  /// \param distance Distance to move, in meters.
  /// \param interpolator Interpolator used to produce the output trajectory.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToEndEffectorOffset(
      statespace::StateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bodyNode,
      const statespace::StateSpace::State* startState,
      const Eigen::Vector3d& direction,
      double distance,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Sets body node or robot for which the path is to be planned.
  void setBodyNode(dart::dynamics::BodyNodePtr bodyNode);

  /// Returns the body node or robot for which the path is to be planned.
  dart::dynamics::BodyNodePtr getBodyNode();

  /// Sets start state.
  void setStartState(const statespace::StateSpace::State* startState);

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Sets direction of motion specified in the world frame.
  void setDirection(const Eigen::Vector3d& direction);

  /// Returns the direction of motion specified in the world frame.
  const Eigen::Vector3d& getDirection() const;

  /// Sets distance in meters to move in the specified direction.
  void setDistance(double distance);

  /// Returns the distance in meters to move in the specified direction.
  double getDistance() const;

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// Body Node or Robot.
  dart::dynamics::BodyNodePtr mBodyNode;

  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Direction of motion.
  Eigen::Vector3d mDirection;

  /// Distance to move in the direction specified.
  double mDistance;

  /// Trajectory-wide constraint that must be satisfied.
  constraint::ConstTestablePtr mConstraint;
};

} // namespace planner
} // namespace aikido

#endif
