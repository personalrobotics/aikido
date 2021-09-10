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
  /// Constructor. Note that this constructor takes the start state from the
  /// current state of the passed MetaSkeleton.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] metaSkeleton MetaSkeleton that getStartState will return the
  /// current state of when called.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// TSR.
  /// \param[in] maxSamplingTries Maximum number of times to try when
  /// sampling from the TSR.
  /// \param[in] batchSize Number of TSR samples to include per batch.
  /// \param[in] maxBatches Maximum number of batches to run when planning to
  /// TSR samples.
  /// \param[in] goalTSR Goal TSR.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToTSR(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      std::size_t maxSamplingTries,
      std::size_t batchSize,
      std::size_t maxBatches,
      constraint::dart::ConstTSRPtr goalTSR,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Constructor. Note that this constructor sets the start state on
  /// construction.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] startState Start state to plan from.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// TSR.
  /// \param[in] maxSamplingTries Maximum number of times to try when
  /// sampling from the TSR.
  /// \param[in] batchSize Number of TSR samples to include per batch.
  /// \param[in] maxBatches Maximum number of batches to run when planning to
  /// TSR samples.
  /// \param[in] goalTSR Goal TSR.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToTSR(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      const statespace::dart::MetaSkeletonStateSpace::State* startState,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      std::size_t maxSamplingTries,
      std::size_t batchSize,
      std::size_t maxBatches,
      constraint::dart::ConstTSRPtr goalTSR,
      constraint::ConstTestablePtr constraint = nullptr);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the end-effector BodyNode to be planned to move to a desired TSR.
  ::dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Returns the maximum number of times to try when sampling from the TSR.
  std::size_t getMaxSamplingTries() const;

  /// Returns the number of samples from the TSR to include per batch.
  std::size_t getBatchSize() const;

  /// Returns the maximum number of batches to run when planning to TSR samples.
  std::size_t getMaxBatches() const;

  /// Returns the start state to plan from, either set on construction or
  /// taken from the current state of the MetaSkeleton.
  const statespace::dart::MetaSkeletonStateSpace::State* getStartState() const;

  /// Returns the goal TSR.
  constraint::dart::ConstTSRPtr getGoalTSR() const;

protected:
  /// MetaSkeletonStateSpace. Prevents use of expensive dynamic cast on
  /// mStateSpace.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// MetaSkeleton, if given.
  ::dart::dynamics::ConstMetaSkeletonPtr mMetaSkeleton;

  /// Start state, if set on construction.
  statespace::dart::MetaSkeletonStateSpace::ScopedState mStartState;

  /// End-effector body node.
  const ::dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;

  /// Maximum number of times to try when sampling from the TSR.
  std::size_t mMaxSamplingTries;

  /// Number of TSR samples to include per batch.
  std::size_t mBatchSize;

  /// Maximum number of batches to run when planning to TSR samples.
  std::size_t mMaxBatches;

  /// Goal TSR.
  const constraint::dart::ConstTSRPtr mGoalTSR;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOTSR_HPP_
