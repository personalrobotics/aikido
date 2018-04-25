#ifndef AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATION_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATION_HPP_

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a single goal configuration.
class ConfigurationToConfiguration : public Problem
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space that this problem associated with.
  /// \param[in] startState Start state.
  /// \param[in] goalState Goal state.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToConfiguration(
      const statespace::ConstStateSpacePtr& stateSpace,
      const statespace::StateSpace::State* startState,
      const statespace::StateSpace::State* goalState,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Sets the start state.
  void setStartState(const statespace::StateSpace::State* state);

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Sets the goal state.
  void setGoalState(const statespace::StateSpace::State* state);

  /// Returns the goal state.
  const statespace::StateSpace::State* getGoalState() const;

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal state.
  const statespace::StateSpace::State* mGoalState;

  /// Trajectory-wide constraint that must be satisfied.
  constraint::ConstTestablePtr mConstraint;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATION_HPP_
