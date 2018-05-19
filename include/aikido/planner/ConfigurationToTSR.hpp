#ifndef AIKIDO_PLANNER_CONFIGURATIONTOTSR_HPP_
#define AIKIDO_PLANNER_CONFIGURATIONTOTSR_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a given single Task Space Region (TSR).
class ConfigurationToTSR : public Problem
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] metaSkeleton MetaSkeleton to plan with.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// TSR.
  /// \param[in] startState Start state.
  /// \param[in] goalTSR Goal TSR.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToTSR(
      statespace::ConstStateSpacePtr stateSpace,
      dart::dynamics::MetaSkeletonPtr metaSkeleton,
      dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      const statespace::StateSpace::State* startState,
      constraint::dart::ConstTSRPtr goalTSR,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the MetaSkeleton to plan with.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton();

  /// Returns the MetaSkeleton to plan with.
  dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const;

  /// Returns the end-effector BodyNode to be planned to move to a desired TSR.
  dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Returns the start state.
  const statespace::StateSpace::State* getStartState() const;

  /// Returns the goal TSR.
  constraint::dart::ConstTSRPtr getGoalTSR() const;

protected:
  /// MetaSkeleton.
  const dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  /// End-effector body node.
  const dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;

  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal TSR
  const constraint::dart::ConstTSRPtr mGoalTSR;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_CONFIGURATIONTOTSR_HPP_
