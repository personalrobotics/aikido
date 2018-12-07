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
      statespace::ConstStateSpacePtr stateSpace,
      const statespace::StateSpace::State* startState,
      const statespace::StateSpace::State* goalState,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  // Documentation inherited.
  std::shared_ptr<Problem> clone() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Returns the goal state.
  const statespace::StateSpace::State* getGoalState() const;

protected:
  /// Start state.
  statespace::StateSpace::ScopedState mStartState;

  /// Goal state.
  statespace::StateSpace::ScopedState mGoalState;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOCONFIGURATION_HPP_
