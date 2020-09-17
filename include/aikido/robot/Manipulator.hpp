#ifndef AIKIDO_ROBOT_MANIPULATOR_HPP_
#define AIKIDO_ROBOT_MANIPULATOR_HPP_

#include "aikido/robot/Hand.hpp"
#include "aikido/robot/Robot.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Manipulator)

/// Base interface for manipulator.
/// A manipulator has a hand.
class Manipulator : public Robot
{
public:
  /// Constructor.
  /// \param[in] name Name of the robot.
  /// \param[in] metaSkeleton Metaskeleton of the robot.
  /// \param[in] hand Hand associated with this manipulator.
  /// \param[in] rng Random number generator.
  /// \param[in] trajectoryExecutor Trajectory executor for the metaSkeleton.
  /// \param[in] collisionDetector Collision detector.
  /// \param[in] selfCollisionFilter Collision filter for self collision.
  Manipulator(
      const std::string& name,
      dart::dynamics::MetaSkeletonPtr metaSkeleton,
      HandPtr hand,
      aikido::common::UniqueRNGPtr rng,
      aikido::control::TrajectoryExecutorPtr trajectoryExecutor,
      dart::collision::CollisionDetectorPtr collisionDetector,
      std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
          selfCollisionFilter);

  /// Destructor.
  virtual ~Manipulator() = default;

  /// Returns the [const] hand.
  ConstHandPtr getHand() const;

  /// Returns the hand.
  HandPtr getHand();

  // Documentation inherited.
  void step(const std::chrono::system_clock::time_point& timepoint) override;

private:
  /// Hand of this manipulator.
  HandPtr mHand;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_MANIPULATOR_HPP_
