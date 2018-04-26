#ifndef AIKIDO_PLANNER_PLANTOCONFIGURATIONS_HPP_
#define AIKIDO_PLANNER_PLANTOCONFIGURATIONS_HPP_

#include <unordered_set>
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/Interpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to multiple goal configurations.
///
/// Plan a trajectory from start state to any of the goal states using an
/// interpolator to interpolate between the states.
class ConfigurationToConfigurations : public Problem
{
public:
  using GoalStates = std::unordered_set<const statespace::StateSpace::State*>;

  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param startState Start state.
  /// \param goalStates Goal states.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToConfigurations(
      statespace::StateSpacePtr stateSpace,
      const statespace::StateSpace::State* startState,
      const GoalStates& goalStates,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Sets start state.
  void setStartState(statespace::StateSpace::State* startState);

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Sets vector of goal states.
  void setGoalStates(const GoalStates& goalStates);

  /// Add a goal state, if this class doesn't already contain the goal state.
  void addGoalState(const statespace::StateSpace::State* goalState);

  /// Returns the vector of goal states.
  const GoalStates& getGoalStates() const;

  /// Sets constraint that must be satisfied throughout the trajectory.
  void setConstraint(constraint::ConstTestablePtr constraint);

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal States.
  GoalStates mGoalStates;

  /// Trajectory-wide constrained that must be satisfied.
  constraint::ConstTestablePtr mConstraint;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANTOCONFIGURATIONS_HPP_
