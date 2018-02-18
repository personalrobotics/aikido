#ifndef AIKIDO_ROBOT_CONCRETEMANIPULATOR_HPP_
#define AIKIDO_ROBOT_CONCRETEMANIPULATOR_HPP_

#include "aikido/robot/Hand.hpp"
#include "aikido/robot/Manipulator.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(ConcreteManipulator)

/// A concrete implementation of a manipulator.
/// ConcreteManipulator supports a collection of
/// end-effector specific planning methods.
class ConcreteManipulator : public Manipulator
{
public:
  ConcreteManipulator(RobotPtr robot, HandPtr hand);

  virtual ~ConcreteManipulator() = default;

  /// \copydoc Manipulator::getHand()
  virtual HandPtr getHand() override;

  /// Plan to a desired end-effector offset with fixed orientation.
  /// \param[in] space The StateSpace for the metaskeleton.
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] body Bodynode for the end effector.
  /// \param[in] collisionFree CollisionFree constraint to check.
  /// Self-collision is checked by default.
  /// \param[in] direction Direction unit vector in the world frame.
  /// \param[in] distance Distance distance to move, in meters.
  /// \param[in] linearVelocity Velocity to move
  /// \return Output trajectory
  trajectory::TrajectoryPtr planToEndEffectorOffset(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& body,
      const constraint::CollisionFreePtr& collisionFree,
      const Eigen::Vector3d& direction,
      double distance,
      double linearVelocity);

  /// Get the direction of an end effector (along z axis) in the world frame
  /// \param body Bodynode for the end effector
  /// \return Output The direction of the end effector (z axis of the frame)
  Eigen::Vector3d getEndEffectorDirection(
      const dart::dynamics::BodyNodePtr& body) const;

  /// Plan to a desired end-effector offset along the z axis of the
  /// end-effector.
  /// \param[in] space The StateSpace for the metaskeleton.
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] body Bodynode for the end effector.
  /// \param[in] collisionFree CollisionFree constraint to check.
  /// Self-collision is checked by default.
  /// \param[in] distance Distance distance to move, in meters.
  /// \param[in] linearVelocity Velocity to move
  /// \return Output trajectory
  trajectory::TrajectoryPtr planEndEffectorStraight(
      statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& body,
      const constraint::CollisionFreePtr& collisionFree,
      double distance,
      double linearVelocity);

  /// \copydoc Robot::postprocessPath
  virtual trajectory::TrajectoryPtr postprocessPath(
      const trajectory::TrajectoryPtr& path) override;

  /// \copydoc Robot::executeTrajectory
  virtual void executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) override;

  /// \copydoc Robot::executePath
  virtual void executePath(const trajectory::TrajectoryPtr& path) override;

  /// \copydoc Robot::getNamedConfiguration
  virtual boost::optional<Eigen::VectorXd> getNamedConfiguration(
      const std::string& name) const override;

  /// \copydoc Robot::setNamedConfigurations
  virtual void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations) override;

  /// \copydoc Robot::getName
  virtual std::string getName() const override;

  /// \copydoc Robot::getMetaSkeleton
  virtual dart::dynamics::MetaSkeletonPtr getMetaSkeleton() override;

  /// \copydoc Robot::setRoot
  virtual void setRoot(Robot* robot) override;

  /// \copydoc Robot::step
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override;

  /// \copydoc Robot::getSelfCollisionConstraint
  virtual aikido::constraint::CollisionFreePtr getSelfCollisionConstraint()
      override;

  /// \copydoc Robot::getFullSelfCollisionConstraint
  virtual aikido::constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const constraint::CollisionFreePtr& collisionFree) override;

private:
  RobotPtr mRobot;
  HandPtr mHand;
};

} // namespace robot
} // namespace aikido

#endif
