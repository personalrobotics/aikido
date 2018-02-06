#ifndef AIKIDO_ROBOT_MANIPULATOR_HPP_
#define AIKIDO_ROBOT_MANIPULATOR_HPP_

#include <dart/dart.hpp>
#include <string>
#include <aikido/robot/Robot.hpp>
#include <aikido/robot/Hand.hpp>

namespace aikido {
namespace robot {

/// A base class for a manipulator which has a hand.
class Manipulator : public Robot
{
public:

  /// Clones this Robot.
  /// \param newName New name for this robot
  virtual std::unique_ptr<Robot> clone(const std::string &newName) override;

  std::unique_ptr<Hand> getHand();

  /// Plan to a desired end-effector offset with fixed orientation.
  /// \param space The StateSpace for the metaskeleton
  /// \param body Bodynode for the end effector
  /// \param direction Direction unit vector in the world frame
  /// \param distance Distance distance to move, in meters
  /// \param maxNumTrials Max number of trials for solving for IK
  /// \param timelimit Max time (seconds) to spend per planning
  /// \param collisionFree CollisionFree constraint to check. Self-collision
  /// is checked by default.
  /// \param positionTolerance Position tolerance for constraint TSR
  /// \param angularTolerance Angular tolerance for constraint TSR
  /// \return Output trajectory
  virtual trajectory::TrajectoryPtr planToEndEffectorOffset(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &body,
      const Eigen::Vector3d &direction,
      const constraint::CollisionFreePtr &collisionFree,
      double distance,
      double linearVelocity,
      int maxNumTrials,
      double timelimit,
      double positionTolerance = 1e-3,
      double angularTolerance = 1e-3,
      double linearGain = 1.0,
      double angularGain = 0.2,
      double timestep = 0.1);


  /// Get the direction of an end effector (along z axis) in the world frame
  /// \param bodynode Bodynode for the end effector
  /// \return Output The direction of the end effector (z axis of the frame)
  Eigen::Vector3d getEndEffectorDirection(
      const dart::dynamics::BodyNodePtr &body) const;

  /// Plan to a desired end-effector offset along the z axis of the
  /// end-effector. Tries VF and then CRRT.
  /// \return Output trajectory
  virtual trajectory::TrajectoryPtr planEndEffectorStraight(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &body,
      const constraint::CollisionFreePtr &collisionFree,
      double distance,
      double linearVelocity,
      int maxNumTrials,
      double timelimit,
      double positionTolerance = 1e-3,
      double angularTolerance = 1e-3,
      double linearGain = 1.0,
      double angularGain = 0.2,
      double timestep = 0.1);

protected:

  Hand mHand;
};

using ManipulatorPtr = std::shared_ptr<Manipulator>;

} // namespace robot
} // namespace aikido
