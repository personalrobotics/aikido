#ifndef AIKIDO_ROBOT_CONCRETEMANIPULATOR_HPP_
#define AIKIDO_ROBOT_CONCRETEMANIPULATOR_HPP_

#include "aikido/robot/Hand.hpp"
#include "aikido/robot/Manipulator.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(ConcreteManipulator)

class ConcreteManipulator : public Manipulator
{
public:
  ConcreteManipulator(RobotPtr robot, HandPtr hand);

  virtual ~ConcreteManipulator() = default;

  virtual HandPtr getHand() override;

  trajectory::TrajectoryPtr planToEndEffectorOffset(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& body,
      const Eigen::Vector3d& direction,
      const constraint::CollisionFreePtr& collisionFree,
      double distance,
      double linearVelocity);

  Eigen::Vector3d getEndEffectorDirection(
      const dart::dynamics::BodyNodePtr& body) const;

  trajectory::TrajectoryPtr planEndEffectorStraight(
      statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& body,
      const constraint::CollisionFreePtr& collisionFree,
      double distance,
      double linearVelocity);

  /// Returns a timed trajectory that can be executed by the robot
  /// \param path Geometric path to execute
  virtual trajectory::TrajectoryPtr postprocessPath(
      const trajectory::TrajectoryPtr& path) override;

  /// Executes a trajectory
  /// \param trajectory Timed trajectory to execute
  virtual void executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) override;

  /// Postprocesses and executes a path
  /// \param timelimit Timelimit for postprocessing.
  virtual void executePath(const trajectory::TrajectoryPtr& path) override;

  // Returns a named configuration
  virtual boost::optional<Eigen::VectorXd> getNamedConfiguration(
      const std::string& name) const override;

  /// Sets the list of named configurations
  /// \param namedConfigurations Map of name, configuration
  virtual void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations) override;

  /// \return Name of this Robot
  virtual std::string getName() const override;

  /// Returns the MetaSkeleton of this robot.
  virtual dart::dynamics::MetaSkeletonPtr getMetaSkeleton() override;

  /// Sets the root of this robot.
  virtual void setRoot(Robot* robot) override;

  /// Simulates up to the provided timepoint.
  /// Assumes that parent robot is locked.
  /// \param timepoint Time to simulate to.
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override;

  virtual aikido::constraint::CollisionFreePtr getSelfCollisionConstraint()
      override;

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
