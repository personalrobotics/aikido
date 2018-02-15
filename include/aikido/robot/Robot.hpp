#ifndef AIKIDO_ROBOT_ROBOT_HPP_
#define AIKIDO_ROBOT_ROBOT_HPP_

#include <chrono>
#include <string>
#include <unordered_map>
#include <Eigen/Core>
#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/constraint/CollisionFree.hpp"
#include "aikido/constraint/TSR.hpp"
#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/RNG.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

/// A base class for robots.
/// A robot has single metaSkeleton.
/// All planning calls made on this robot should be made on a subset of
/// this robot's metaSkeleton.
class Robot
{
public:

  /// Constructor.
  /// \param name Name of the robot.
  /// \param robot Metaskeleton of the robot.
  /// \param statespace StateSpace this robot belongs to.
  /// \param simulation True if running in simulation.
  /// \param rngSeed seed for initializing random generator.
  /// \param trajectoryExecutor Trajectory executor.
  /// \param retimer Postprocessor used for retiming paths.
  /// \param smoother Postprocessor used for smoothing paths.
  /// \param collisionResolution Resolution to check collision.
  Robot::Robot(
    const std::string& name,
    MetaSkeletonPtr robot,
    MetaSkeletonStateSpacePtr statespace,
    bool simulation,
    aikido::common::RNG::result_type rngSeed,
    std::unique_ptr<control::TrajectoryExecutor> trajectoryExecutor,
    std::unique_ptr<planner::parabolic::ParabolicTimer> retimer,
    std::unique_ptr<planner::parabolic::ParabolicSmoother> smoother,
    double collisionResolution);

  virtual ~Robot();

  /// Plans a trajectory to the specified configuration
  virtual trajectory::TrajectoryPtr planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State *startState,
    const statespace::StateSpace::State *goalState,
    const constraint::CollisionFreePtr &collisionFree);

  /// Wrapper for planToConfiguration using Eigen vectors.
  virtual trajectory::TrajectoryPtr planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const Eigen::VectorXd &goal,
    const constraint::CollisionFreePtr &collisionFree);

  /// Plans a trajectory to one of the spcified configurations.
  virtual trajectory::TrajectoryPtr planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State *startState,
    const std::vector<statespace::StateSpace::State*> &goalStates,
    const constraint::CollisionFreePtr &collisionFree);

  /// Wrapper for planToConfigurations using Eigen vectors.
  virtual trajectory::TrajectoryPtr planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const std::vector<Eigen::VectorXd> &goals,
    const constraint::CollisionFreePtr &collisionFree);

  /// Plans to a TSR.
  virtual trajectory::TrajectoryPtr planToTSR(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State *startState,
    const constraint::TSR &tsr,
    const constraint::CollisionFreePtr &collisionFree);

  /// Wrapper for planToTSR using Eigen vectors.
  virtual trajectory::TrajectoryPtr planToTSR(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const constraint::TSR &tsr,
    const constraint::CollisionFreePtr &collisionFree);

  /// Returns a Trajectory that moves the configuration of the metakeleton such
  /// that the specified bodynode is set to a sample in a goal TSR and
  /// the trajectory is constrained to a constraint TSR
  /// \param space The StateSpace for the metaskeleton
  /// \param body Bodynode whose frame is meant for TSR
  /// \param goalTsr The goal TSR to move to
  /// \param constraintTsr The constraint TSR for the trajectory
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  virtual trajectory::TrajectoryPtr
  planToTSRwithTrajectoryConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &bodyNode,
      const constraint::TSRPtr &goalTsr,
      const constraint::TSRPtr &constraintTsr,
      const constraint::CollisionFreePtr &collisionFree);

  /// Plans to a named configuration.
  /// \param startState Starting state
  /// \param name Name of the configuration to plan to
  /// \param collisionFree Collision constraint
  /// \return Trajectory to the configuration, or nullptr if planning fails
  virtual trajectory::TrajectoryPtr planToNamedConfiguration(
    const statespace::StateSpace::State* startState,
    const std::string &name,
    const CollisionFreePtr &collisionFree);

  /// Returns a timed trajectory that can be executed by the robot
  /// \param path Geometric path to execute
  virtual trajectory::TrajectoryPtr postprocessPath(
    const trajectory::TrajectoryPtr &path);

  /// Executes a trajectory
  /// \param trajectory Timed trajectory to execute
  virtual void executeTrajectory(
    const trajectory::TrajectoryPtr &trajectory);

  /// Postprocesses and executes a path
  /// \param timelimit Timelimit for postprocessing.
  virtual void executePath(
    const trajectory::TrajectoryPtr &path);

  // Returns a named configuration
  /// \throws invalid_argument error if name does not exist.
  Eigen::VectorXd getNamedConfiguration(
    const std::string &name);

  // Returns the configuration of the whole robot
  Eigen::VectorXd getConfiguration();

  /// \return Name of this Robot
  std::string getName();

  /// Returns the MetaSkeleton of this robot.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton();

  void setRoot(Robot *robot);

  // Simulates up to the provided timepoint.
  // \param timepoint Time to simulate to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint);

  // Calls step with current time.
  void update();

  // Clones RNG
  std::unique_ptr<common::RNG> cloneRNG();

protected:

  struct Configuration
  {
    const statespace::MetaSkeletonStateSpacePtr metaSkeletonStateSpace;
    const dart::dynamics::dart::dynamics::MetaSkeletonPtr metaSkeleton;
    const Eigen::VectorXd position;
  };

  /// Returns true if this Robot contains this metaSkeleton.
  virtual bool checkIfMetaSkeletonBelongs(
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton);

  constraint::CollisionFreePtr createSelfCollisionConstraint() const;

  constraint::CollisionFreePtr getSelfCollisionConstraint() const;

  constraint::TestablePtr getCollisionConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const constraint::CollisionFreePtr &collisionFree) const;

  /// Sets the root of this robot.
  void setRoot(Robot* robot);

  /// If this robot belongs to another (Composite)Robot,
  /// mRootRobot is the topmost robot containing this robot.
  Robot* mRootRobot;

  /// Name of this robot
  std::string mName;

  dart::dynamics::MetaSkeletonPtr mRobot;
  statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;

  // True if running in simulation mode
  bool mSimulation;

  /// Random generator
  common::RNGWrapper<std::mt19937> mRng;

  std::shared_ptr<control::TrajectoryExecutor> mTrajectoryExecutor;

  std::unique_ptr<planner::parabolic::ParabolicTimer> mRetimer;

  std::unique_ptr<planner::parabolic::ParabolicSmoother> mSmoother;

  double mCollisionResolution;

  /// Commonly used configurations.
  std::unordered_map<std::string, Configuration> mNamedConfigurations;

  dart::collision::CollisionGroupPtr mCollideWith;
  dart::collision::CollisionDetectorPtr mCollisionDetector;
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;

  constraint::CollisionFree::CollisionFreePtr mSelfCollisionConstraint;

};

using RobotPtr = std::shared_ptr<Robot>;

} // namespace robot
} // namespace aikido

#endif

