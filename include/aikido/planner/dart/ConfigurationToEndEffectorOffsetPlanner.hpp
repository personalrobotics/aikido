#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTOROFFSETPLANNER_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTOROFFSETPLANNER_HPP_

#include "aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp"
#include "aikido/planner/dart/SingleProblemPlanner.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Base planner class for ConfigurationToEndEffectorOffset planning problem.
class ConfigurationToEndEffectorOffsetPlanner
    : public dart::SingleProblemPlanner<ConfigurationToEndEffectorOffsetPlanner,
                                        ConfigurationToEndEffectorOffset>
{
public:
  // Expose the implementation of Planner::plan(const Problem&, Result*) in
  // SingleProblemPlanner. Note that plan() of the base class takes Problem
  // while the virtual function defined in this class takes SolvableProblem,
  // which is simply ConfigurationToEndEffectorOffset.
  using SingleProblemPlanner::plan;

  /// Constructor
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] metaSkeleton MetaSkeleton to use for planning.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// offest while maintaining the current orientation. Current direction is
  /// returned when getDirection is called.
  ConfigurationToEndEffectorOffsetPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode = nullptr);

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  virtual trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr)
      = 0;
  // Note: SolvableProblem is defined in SingleProblemPlanner.

  /// Returns the end-effector BodyNode to be planned to move a desired offest
  /// while maintaining the current orientation.
  ::dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Sets the EndEffectorBodyNode.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// TSR.
  void setEndEffectorBodyNode(
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode);

protected:

  /// End-effector body node.
  ::dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;

};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTOROFFSETPLANNER_HPP_
