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
      const std::unordered_set<statespace::StateSpace::State*>& goalStates,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Returns the vector of goal states.
  const std::unordered_set<statespace::StateSpace::State*>& getGoalStates()
      const;

  /// Returns the constraint that must be satisfied throughout the trajectory.
  constraint::ConstTestablePtr getConstraint() const;

protected:
  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal States.
  std::unordered_set<statespace::StateSpace::State*> mGoalStates;

  /// Trajectory-wide constrained that must be satisfied.
  constraint::ConstTestablePtr mConstraint;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANTOCONFIGURATIONS_HPP_
