#ifndef AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTOROFFSET_HPP_
#define AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTOROFFSET_HPP_

#include <boost/optional.hpp>
#include <dart/dart.hpp>
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Planning problem to plan to desired end-effector offset while maintaining
/// the current end-effector orientation. Each constructor makes two choices:
/// whether to take in a direction, and taking in either a start state OR a
/// MetaSkeleton. If a direction is not passed, getDirection returns the current
/// direction of the end effector body node. If a MetaSkeleton is passed instead
/// of a start state, getStartState will return the current state of that
/// MetaSkeleton. Thus, there are 2x2=4 constructors here.
class ConfigurationToEndEffectorOffset : public Problem
{
public:
  /// Constructor. Note that this constructor takes the start state from the
  /// current state of the passed MetaSkeleton, and sets the direction on
  /// construction.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] metaSkeleton MetaSkeleton that getStartState will return the
  /// current state of when called.
  /// \param[in] signedDistance Signed distance to move, in meters.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If the size of \c direction is zero.
  ConfigurationToEndEffectorOffset(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      double signedDistance,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Constructor. Note that this constructor sets the start state and direction
  /// on construction.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] startState Start state to plan from.
  /// \param[in] signedDistance Signed distance to move, in meters.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If the size of \c direction is zero.
  ConfigurationToEndEffectorOffset(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      const statespace::dart::MetaSkeletonStateSpace::State* startState,
      double signedDistance,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Constructor. Note that this constructor takes the start state from the
  /// current state of the passed MetaSkeleton, and takes the direction from the
  /// end-effector BodyNode's current direction.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] metaSkeleton MetaSkeleton that getStartState will return the
  /// current state of when called.
  /// \param[in] direction Unit vector that represents the direction of motion
  /// [unit vector in the world frame].
  /// \param[in] signedDistance Signed distance to move, in meters.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  ConfigurationToEndEffectorOffset(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const Eigen::Vector3d& direction,
      double signedDistance,
      constraint::ConstTestablePtr constraint = nullptr);

  /// Constructor. Note that this constructor sets the start state on
  /// construction, and takes the direction from the end-effector BodyNode's
  /// current direction.
  ///
  /// \param[in] stateSpace State space.
  /// \param[in] startState Start state to plan from.
  /// \param[in] direction Unit vector that represents the direction of motion
  /// [unit vector in the world frame].
  /// \param[in] signedDistance Signed distance to move, in meters.
  /// \param[in] constraint Trajectory-wide constraint that must be satisfied.
  ConfigurationToEndEffectorOffset(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      const statespace::dart::MetaSkeletonStateSpace::State* startState,
      const Eigen::Vector3d& direction,
      double signedDistance,
      constraint::ConstTestablePtr constraint = nullptr);

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns the type of the planning problem.
  static const std::string& getStaticType();

  /// Returns the end-effector BodyNode to be planned to move a desired offest
  /// while maintaining the current orientation.
  //::dart::dynamics::ConstBodyNodePtr getEndEffectorBodyNode() const;

  /// Returns the start state to plan from, either set on construction or
  /// taken from the current state of the MetaSkeleton.
  const statespace::dart::MetaSkeletonStateSpace::State* getStartState() const;

  /// Returns the direction of motion specified in the world frame.
  boost::optional<Eigen::Vector3d> getDirection() const;

  /// Returns the signed distance in meters to move in the specified direction.
  double getDistance() const;

  /*
  // Documentation inherited.
  std::shared_ptr<Problem> clone(
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton) const override;
  */

protected:
  /// MetaSkeletonStateSpace. Prevents use of expensive dynamic cast on
  /// mStateSpace.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// MetaSkeleton, if given.
  ::dart::dynamics::ConstMetaSkeletonPtr mMetaSkeleton;

  /// Start state, if set on construction.
  statespace::dart::MetaSkeletonStateSpace::ScopedState mStartState;

  /// End-effector body node.
  //const ::dart::dynamics::ConstBodyNodePtr mEndEffectorBodyNode;

  /// Direction of motion, if set on construction.
  const boost::optional<Eigen::Vector3d> mDirection;

  /// Signed distance to move in the direction specified.
  const double mDistance;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CONFIGURATIONTOENDEFFECTOROFFSET_HPP_
