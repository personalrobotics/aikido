#ifndef AIKIDO_ROBOT_MANIPULATOR_HPP_
#define AIKIDO_ROBOT_MANIPULATOR_HPP_

#include <string>
#include <dart/dart.hpp>
#include "aikido/robot/Hand.hpp"
#include "aikido/robot/Robot.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Manipulator)

/// A base class for a manipulator which has a hand.
class Manipulator : public Robot
{
public:
  Manipulator(
      const std::string& name,
      dart::dynamics::MetaSkeletonPtr robot,
      statespace::dart::MetaSkeletonStateSpacePtr statespace,
      bool simulation,
      common::RNG::result_type rngSeed,
      std::unique_ptr<control::TrajectoryExecutor> executor,
      std::unique_ptr<planner::parabolic::ParabolicTimer> retimer,
      std::unique_ptr<planner::parabolic::ParabolicSmoother> smoother,
      std::shared_ptr<Hand> hand);

  virtual ~Manipulator() = default;

  std::shared_ptr<Hand> getHand();

  /// Plan to a desired end-effector offset with fixed orientation.
  /// \param space StateSpace for the metaskeleton
  /// \param body Bodynode for the end effector
  /// \param metaSkeleton Metaskeleton to plan with
  /// \param direction Direction unit vector in the world frame
  /// \param distance Distance distance to move, in meters
  /// \param collisionFree CollisionFree constraint to check. Self-collision
  /// is checked by default.
  /// \return Output trajectory
  virtual trajectory::TrajectoryPtr planToEndEffectorOffset(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& body,
      const Eigen::Vector3d& direction,
      const constraint::CollisionFreePtr& collisionFree,
      double distance,
      double linearVelocity);

  /// Get the direction of an end effector (along z axis) in the world frame
  /// \param bodynode Bodynode for the end effector
  /// \return Output The direction of the end effector (z axis of the frame)
  Eigen::Vector3d getEndEffectorDirection(
      const dart::dynamics::BodyNodePtr& body) const;

  /// Plan to a desired end-effector offset along the z axis of the
  /// end-effector. Tries VF and then CRRT.
  /// \return Output trajectory
  virtual trajectory::TrajectoryPtr planEndEffectorStraight(
      statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& body,
      const constraint::CollisionFreePtr& collisionFree,
      double distance,
      double linearVelocity);

  // Documentation inherited.
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override;

protected:
  std::shared_ptr<Hand> mHand;
};

} // namespace robot
} // namespace aikido

#endif
