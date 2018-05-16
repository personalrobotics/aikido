#ifndef AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONS_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONS_HPP_

#include <unordered_set>
#include <set>
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
  using ScopedGoalStates = std::set<statespace::StateSpace::ScopedStateConst>;
  // TODO(JS): Use `std::unordered_set`.

  /// Constructor.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] startState Start state.
  /// \param[in] goalStates Goal states.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToConfigurations(
      statespace::ConstStateSpacePtr stateSpace,
      const statespace::StateSpace::State* startState,
      const GoalStates& goalStates,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Returns the number of the goal states.
  std::size_t getNumGoalStates() const;

  /// Returns goal states.
  const ScopedGoalStates& getGoalStates() const;

protected:
  /// Start state.
  statespace::StateSpace::ScopedStateConst mStartState;

  /// Goal States.
  ScopedGoalStates mGoalStates;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATIONS_HPP_
