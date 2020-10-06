#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONS_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONS_HPP_

#include <dart/dart.hpp>

#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Planning problem to plan to multiple goal configurations.
class ConfigurationToConfigurations : public Problem
{
public:
  using GoalStates
      = std::vector<const statespace::dart::MetaSkeletonStateSpace::State*>;

  /// Constructor. Note that this constructor takes the start state from the
  /// current state of the passed MetaSkeleton.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] metaSkeleton MetaSkeleton that getStartState will return the
  /// current state of when called.
  /// \param[in] goalStates Goal states.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  ConfigurationToConfigurations(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const GoalStates& goalStates,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Constructor. Note that this constructor sets the start state on
  /// construction.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] startState Start state to plan from.
  /// \param[in] goalStates Goal states.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  ConfigurationToConfigurations(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      const statespace::dart::MetaSkeletonStateSpace::State* startState,
      const GoalStates& goalStates,
      constraint::ConstTestablePtr constraint = nullptr);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the start state.
  const statespace::dart::MetaSkeletonStateSpace::State* getStartState() const;

  /// Returns the number of the goal states.
  std::size_t getNumGoalStates() const;

  /// Returns goal states.
  GoalStates getGoalStates() const;

protected:
  /// MetaSkeletonStateSpace. Prevents use of expensive dynamic cast on
  /// mStateSpace.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// MetaSkeleton, if given.
  ::dart::dynamics::ConstMetaSkeletonPtr mMetaSkeleton;

  /// Start state.
  statespace::dart::MetaSkeletonStateSpace::ScopedState mStartState;

  /// Goal States.
  std::vector<statespace::dart::MetaSkeletonStateSpace::ScopedState>
      mGoalStates;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATIONS_HPP_
