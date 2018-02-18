#ifndef AIKIDO_ROBOT_CONCRETEROBOT_HPP_
#define AIKIDO_ROBOT_CONCRETEROBOT_HPP_

#include <chrono>
#include <string>
#include <unordered_map>
#include <Eigen/Core>
#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/CollisionFree.hpp"
#include "aikido/constraint/TSR.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"
#include "aikido/robot/Robot.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(ConcreteRobot)

class ConcreteRobot : public Robot
{
public:
  ConcreteRobot(
      const std::string& name,
      dart::dynamics::MetaSkeletonPtr robot,
      statespace::dart::MetaSkeletonStateSpacePtr statespace,
      bool simulation,
      std::unique_ptr<aikido::common::RNG> rng,
      std::shared_ptr<control::TrajectoryExecutor> trajectoryExecutor,
      std::shared_ptr<planner::parabolic::ParabolicTimer> retimer,
      std::shared_ptr<planner::parabolic::ParabolicSmoother> smoother,
      dart::collision::CollisionDetectorPtr collisionDetector,
      dart::collision::CollisionGroupPtr collideWith,
      std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
          selfCollisionFilter);

  virtual ~ConcreteRobot() = default;

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

  /// Plans a trajectory to the specified configuration
  aikido::trajectory::TrajectoryPtr planToConfiguration(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::statespace::StateSpace::State* goalState,
      const aikido::constraint::CollisionFreePtr& collisionFree);

  /// Wrapper for planToConfiguration using Eigen vectors.
  aikido::trajectory::TrajectoryPtr planToConfiguration(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const Eigen::VectorXd& goal,
      const aikido::constraint::CollisionFreePtr& collisionFree);

  /// Plans a trajectory to one of the spcified configurations.
  aikido::trajectory::TrajectoryPtr planToConfigurations(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::vector<aikido::statespace::StateSpace::State*>& goalStates,
      const aikido::constraint::CollisionFreePtr& collisionFree);

  /// Wrapper for planToConfigurations using Eigen vectors.
  aikido::trajectory::TrajectoryPtr planToConfigurations(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::vector<Eigen::VectorXd>& goals,
      const aikido::constraint::CollisionFreePtr& collisionFree);

  /// Plans to a TSR.
  aikido::trajectory::TrajectoryPtr planToTSR(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& bn,
      const aikido::constraint::TSRPtr& tsr,
      const aikido::constraint::CollisionFreePtr& collisionFree);

  /// Returns a Trajectory that moves the configuration of the metakeleton such
  /// that the specified bodynode is set to a sample in a goal TSR and
  /// the trajectory is constrained to a constraint TSR
  /// \param space The StateSpace for the metaskeleton
  /// \param body Bodynode whose frame is meant for TSR
  /// \param goalTsr The goal TSR to move to
  /// \param constraintTsr The constraint TSR for the trajectory
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  aikido::trajectory::TrajectoryPtr planToTSRwithTrajectoryConstraint(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& bodyNode,
      const aikido::constraint::TSRPtr& goalTsr,
      const aikido::constraint::TSRPtr& constraintTsr,
      const aikido::constraint::CollisionFreePtr& collisionFree);

  /// Plans to a named configuration.
  /// \param startState Starting state
  /// \param name Name of the configuration to plan to
  /// \param collisionFree Collision constraint
  /// \return Trajectory to the configuration, or nullptr if planning fails
  aikido::trajectory::TrajectoryPtr planToNamedConfiguration(
      const std::string& name,
      const aikido::constraint::CollisionFreePtr& collisionFree);

  aikido::constraint::CollisionFreePtr getSelfCollisionConstraint() override;

  aikido::constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const constraint::CollisionFreePtr& collisionFree) override;

private:
  // Named Configurations are read from a YAML file
  using ConfigurationMap
      = std::unordered_map<std::string, const Eigen::VectorXd>;

  std::unique_ptr<aikido::common::RNG> cloneRNG();

  aikido::constraint::CollisionFreePtr createSelfCollisionConstraint();

  /// If this robot belongs to another (Composite)Robot,
  /// mRootRobot is the topmost robot containing this robot.
  Robot* mRootRobot;

  /// Name of this robot
  std::string mName;

  dart::dynamics::MetaSkeletonPtr mRobot;
  statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;

  // Skeleton containing mRobot
  dart::dynamics::SkeletonPtr mParentRobot;

  // True if running in simulation mode
  bool mSimulation;

  std::unique_ptr<common::RNG> mRng;

  std::shared_ptr<control::TrajectoryExecutor> mTrajectoryExecutor;

  std::shared_ptr<planner::parabolic::ParabolicTimer> mRetimer;

  std::shared_ptr<planner::parabolic::ParabolicSmoother> mSmoother;

  double mCollisionResolution;

  /// Commonly used configurations.
  ConfigurationMap mNamedConfigurations;

  dart::collision::CollisionDetectorPtr mCollisionDetector;
  dart::collision::CollisionGroupPtr mCollideWith;
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;

  constraint::CollisionFreePtr mSelfCollisionConstraint;
};
}
}

#endif
