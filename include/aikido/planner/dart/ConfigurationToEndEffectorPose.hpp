#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTORPOSE_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTORPOSE_HPP_

#include <dart/dart.hpp>
#include "aikido/planner/dart/DartProblem.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Planning problem to plan to a desired end-effector pose.
class ConfigurationToEndEffectorPose : public DartProblem
{
public:
  /// Constructor. Note that this constructor takes the start state from the
  /// current state of the passed MetaSkeleton.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] metaSkeleton MetaSkeleton that getStartState will return the
  /// current state of when called.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// pose.
  /// \param[in] goalPose Goal pose.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToEndEffectorPose(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      const Eigen::Isometry3d& goalPose,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Constructor. Note that this constructor sets the start state on
  /// construction.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] startState Start state to plan from.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// pose.
  /// \param[in] goalPose Goal pose.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToEndEffectorPose(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      const statespace::dart::MetaSkeletonStateSpace::State* startState,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      const Eigen::Isometry3d& goalPose,
      constraint::ConstTestablePtr constraint = nullptr);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the end-effector BodyNode to be planned to move to a desired pose.
  ::dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Return the start state to plan from, either set on construction or
  /// taken from the current state of the MetaSkeleton.
  const statespace::dart::MetaSkeletonStateSpace::State* getStartState() const;

  /// Returns the goal pose.
  const Eigen::Isometry3d& getGoalPose() const;

  // Documentation inherited.
  std::shared_ptr<Problem> clone(
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton) const override;

protected:
  // Need this due to mGoalPose.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// MetaSkeletonStateSpace. Prevents use of expensive dynamic cast on
  /// mStateSpace.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// MetaSkeleton, if given.
  ::dart::dynamics::ConstMetaSkeletonPtr mMetaSkeleton;

  /// Start state, if set on construction.
  statespace::dart::MetaSkeletonStateSpace::ScopedState mStartState;

  /// End-effector body node.
  const ::dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;

  /// Goal pose.
  const Eigen::Isometry3d mGoalPose;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTORPOSE_HPP_
