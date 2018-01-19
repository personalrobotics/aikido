#ifndef AIKIDO_PLANNER_PLANTOENDEFFECTOROFFSET_HPP_
#define AIKIDO_PLANNER_PLANTOENDEFFECTOROFFSET_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/smart_pointer.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/smart_pointer.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to desired en-effector offset with fixed orientation
class PlanToEndEffectorOffset : public Problem
{
public:
  using ReturnTrajectoryType = trajectory::Interpolated;

  class Result;

  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param bodyNode Body Node or robot for which the path is to be planned.
  /// \param startState Start state.
  /// \param goalState Goal state.
  /// \param direction Direction of motion [unit vector in the world frame].
  /// \param distance Distance to move, in meters.
  /// \param interpolator Interpolator used to produce the output trajectory.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible to \c constraint's state space.
  PlanToEndEffectorOffset(
      statespace::ConstStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bodyNode,
      const statespace::StateSpace::State* startState,
      const statespace::StateSpace::State* goalState,
      const Eigen::Vector3d& direction,
      const double distance,
      statespace::InterpolatorPtr interpolator,
      constraint::TestablePtr constraint);

  const std::string& getName() const override;

  static const std::string& getStaticName();

  dart::dynamics::BodyNodePtr getBodyNode();

  const statespace::StateSpace::State* getStartState() const;

  const statespace::StateSpace::State* getGoalState() const;

  const Eigen::Vector3d& getDirection() const;

  const double getDistance() const;

  statespace::InterpolatorPtr getInterpolator();

  statespace::ConstInterpolatorPtr getInterpolator() const;

  constraint::TestablePtr getConstraint();

  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// Body Node or Robot.
  dart::dynamics::BodyNodePtr mBodyNode;

  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal state.
  const statespace::StateSpace::State* mGoalState;

  /// Direction of motion.
  const Eigen::Vector3d mDirection;

  /// Distance to move in the direction specified.
  const double mDistance;

  /// Interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr mInterpolator;

  /// Trajectory-wide constraint that must be satisfied.
  constraint::TestablePtr mConstraint;
};

class PlanToEndEffectorOffset::Result : public Problem::Result
{
public:
protected:
};

} // namespace planner
} // namespace aikido

#endif
