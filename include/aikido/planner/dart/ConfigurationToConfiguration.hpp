#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATION_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATION_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Planning problem to plan to a single goal configuration.
class ConfigurationToConfiguration : public Problem
{
public:
  /// Constructor. Note that this constructor takes the start state from the
  /// current state of the passed MetaSkeleton.
  ///
  /// \param[in] stateSpace State space.
  /// param[in] metaSkeleton MetaSkeleton that getStartState will return the
  /// current state of when called.
  /// \param[in] goalState Goal state to plan to.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  ConfigurationToConfiguration(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const statespace::dart::MetaSkeletonStateSpace::State* goalState,
      constraint::ConstTestablePtr constraint);

  /// Constructor. Note that this constructor sets the start state on
  /// construction.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] startState Start state to plan from.
  /// \param[in] goalState Goal state to plan to.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  ConfigurationToConfiguration(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      const statespace::dart::MetaSkeletonStateSpace::State* startState,
      const statespace::dart::MetaSkeletonStateSpace::State* goalState,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Returns the goal state.
  const statespace::StateSpace::State* getGoalState() const;

protected:
  /// MetaSkeletonStateSpace. Prevents use of expensive dynamic cast on
  /// mStateSpace.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// MetaSkeleton, if given.
  ::dart::dynamics::ConstMetaSkeletonPtr mMetaSkeleton;

  /// Start state.
  statespace::dart::MetaSkeletonStateSpace::ScopedState mStartState;

  /// Goal state.
  statespace::dart::MetaSkeletonStateSpace::ScopedState mGoalState;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOCONFIGURATION_HPP_
