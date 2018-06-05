#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTORPOSE_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTORPOSE_HPP_

#include <dart/dart.hpp>
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Planning problem to plan to a desired end-effector pose.
class ConfigurationToEndEffectorPose : public Problem
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] endEffectorBodyNode BodyNode to be planned to move to a desired
  /// pose.
  /// \param[in] goalPose Goal pose.
  /// \throw If \c stateSpace is not compatible with \c constraint's state
  /// space.
  ConfigurationToEndEffectorPose(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstBodyNodePtr endEffectorBodyNode,
      const Eigen::Isometry3d& goalPose,
      constraint::ConstTestablePtr constraint);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the end-effector BodyNode to be planned to move to a desired pose.
  ::dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Returns the goal pose.
  const Eigen::Isometry3d& getGoalPose() const;

protected:
  // Need this due to mGoalPose.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// End-effector body node.
  const ::dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;
  /// Goal pose.
  const Eigen::Isometry3d mGoalPose;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTORPOSE_HPP_
