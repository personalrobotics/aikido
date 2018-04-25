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
      constraint::TestablePtr constraint);

  // Documentation inherited.
  const std::string& getName() const override;

  /// Returns the name of the planner problem.
  static const std::string& getStaticName();

  /// Sets the start state.
  void setStartState(const statespace::StateSpace::State* state);

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Sets the goal state.
  void setGoalState(const statespace::StateSpace::State* state);

  /// Returns the goal state.
  const statespace::StateSpace::State* getGoalState() const;

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::TestablePtr getConstraint();

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::TestablePtr getConstraint() const;
  // TODO: Should return ConstInterpolatorPtr when resolving const correctness
  // issues.

protected:
  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal state.
  const statespace::StateSpace::State* mGoalState;

  /// Trajectory-wide constraint that must be satisfied.
  constraint::TestablePtr mConstraint;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATION_HPP_
