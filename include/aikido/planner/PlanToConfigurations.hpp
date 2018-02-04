#ifndef AIKIDO_PLANNER_PLANTOCONFIGURATIONS_HPP_
#define AIKIDO_PLANNER_PLANTOCONFIGURATIONS_HPP_

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/Interpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a multiple goal configurations.
///
/// Plan a trajectory from start state to any of the goal states using an
/// interpolator to interpolate between the states.
class PlanToConfigurations : public Problem
{
public:
  class Result;

  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param startState Start state.
  /// \param goalStates Goal states.
  /// \param interpolator Interpolator used to produce the output trajectory.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible to \c constraint's state space.
  PlanToConfigurations(
      statespace::StateSpacePtr stateSpace,
      const statespace::StateSpace::State* startState,
      const std::vector<statespace::StateSpace::State*>& goalStates,
      statespace::InterpolatorPtr interpolator,
      constraint::TestablePtr constraint);

  /// Returns the name of the planner problem.
  const std::string& getName() const override;
  static const std::string& getStaticName();

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Returns the vector of goal states.
  const std::vector<statespace::StateSpace::State*> getGoalStates() const;

  /// Returns the interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr getInterpolator();

  /// Returns the interpolator used to produce the output trajectory.
  statespace::ConstInterpolatorPtr getInterpolator() const;

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::TestablePtr getConstraint();

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal States.
  const std::vector<statespace::StateSpace::State*> mGoalStates;

  /// Interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr mInterpolator;

  /// Trajectory-wide constrained that must be satisfied.
  constraint::TestablePtr mConstraint;
};

class PlanToConfigurations::Result : public Problem::Result
{
public:
protected:
};

} // namespace planner
} // namespace aikido

#endif
