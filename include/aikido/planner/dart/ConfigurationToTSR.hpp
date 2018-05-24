#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOTSR_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOTSR_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Planning problem to plan to a given single Task Space Region (TSR).
class ConfigurationToTSR : public Problem
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// TSR.
  /// \param[in] maxSamples Maximum number of TSR samples to plan to.
  /// \param[in] startState Start state.
  /// \param[in] goalTSR Goal TSR.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToTSR(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      std::size_t maxSamples,
      const statespace::dart::MetaSkeletonStateSpace::State* startState,
      constraint::dart::ConstTSRPtr goalTSR,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the end-effector BodyNode to be planned to move to a desired TSR.
  ::dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Returns the maximum number of TSR samples to plan to.
  std::size_t getMaxSamples() const;

  /// Returns the start state.
  const statespace::dart::MetaSkeletonStateSpace::State* getStartState() const;

  /// Returns the goal TSR.
  constraint::dart::ConstTSRPtr getGoalTSR() const;

protected:
  /// End-effector body node.
  const ::dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;

  /// Maximum number of TSR samples to plan to.
  std::size_t mMaxSamples;

  /// Start state.
  const statespace::dart::MetaSkeletonStateSpace::State* mStartState;

  /// Goal TSR.
  const constraint::dart::ConstTSRPtr mGoalTSR;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOTSR_HPP_
