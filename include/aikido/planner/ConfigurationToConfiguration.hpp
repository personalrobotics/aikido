#ifndef AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATION_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATION_HPP_

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a single goal configuration.
///
/// Plan a trajectory from start state to goal state by using an interpolator to
/// interpolate between them.
class ConfigurationToConfiguration : public Problem
{
public:
  class Result;

  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param startState Start state.
  /// \param goalState Goal state.
  /// \param interpolator Interpolator used to produce the output trajectory.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible to \c constraint's state space.
  ConfigurationToConfiguration(
      const statespace::ConstStateSpacePtr& stateSpace,
      const statespace::StateSpace::State* startState,
      const statespace::StateSpace::State* goalState,
      statespace::InterpolatorPtr interpolator,
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

  /// Returns the interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr getInterpolator();

  /// Returns the interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr getInterpolator() const;
  // TODO: Should return ConstInterpolatorPtr when resolving const correctness
  // issues.

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

  /// Interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr mInterpolator;

  /// Trajectory-wide constraint that must be satisfied.
  constraint::TestablePtr mConstraint;
};

class ConfigurationToConfiguration::Result : public Problem::Result
{
public:
protected:
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATION_HPP_
