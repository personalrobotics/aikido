#ifndef AIKIDO_PLANNER_PLANTOTSR_HPP_
#define AIKIDO_PLANNER_PLANTOTSR_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a given single Task Space Region (TSR).
class ConfigurationToTSR : public Problem
{
public:
  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param bodyNode Body Node or robot for which the path is to be planned.
  /// \param startState Start state.
  /// \param goalTSR Goal TSR.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToTSR(
      statespace::StateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bodyNode,
      const statespace::StateSpace::State* startState,
      constraint::dart::ConstTSRPtr goalTSR,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Sets BodyNode for which the path is to be planned.
  void setBodyNode(dart::dynamics::BodyNodePtr bodyNode);

  /// Returns BodyNode for which the path is to be planned.
  dart::dynamics::BodyNodePtr getBodyNode();

  /// Sets start state.
  void setStartState(const statespace::StateSpace::State* startState);

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Sets goal TSR.
  void setGoalTSR(constraint::dart::ConstTSRPtr goalTSR);

  /// Returns the goal TSR.
  constraint::dart::ConstTSRPtr getGoalTSR() const;

  /// Sets constraint that must be satisfied throughout the trajectory.
  void setConstraint(constraint::ConstTestablePtr constraint);

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// Body Node or Robot
  dart::dynamics::BodyNodePtr mBodyNode;

  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal TSR
  constraint::dart::ConstTSRPtr mGoalTSR;

  /// Trajectory-wide constraint that must be satisfied.
  constraint::ConstTestablePtr mConstraint;
};

} // namespace planner
} // namespace aikido

#endif
