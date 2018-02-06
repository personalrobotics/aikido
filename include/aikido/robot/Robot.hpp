#ifndef AIKIDO_ROBOT_ROBOT_HPP_
#define AIKIDO_ROBOT_ROBOT_HPP_

#include <string>
#include <unordered_map>
#include <Eigen/Core>
#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/constraint/CollisionFree.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/common/ExecutorThread.hpp>
#include <aikido/common/RNG.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>

namespace aikido {
namespace robot {

/// A base class for robots.
/// A robot has single metaSkeleton.
/// All planning calls made on this robot should be made on a subset of
/// this robot's metaSkeleton.
class Robot
{
public:

  /// Clones this Robot.
  /// \param newName New name for this robot
  virtual std::unique_ptr<Robot> clone(const std::string& newName) = 0;

  /// Plans a trajectory to the specified configuration
  virtual trajectory::TrajectoryPtr planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State *startState,
    const statespace::StateSpace::State *goalState,
    const constraint::CollisionFreePtr &collisionFree,
    double timeLimit);

  /// Wrapper for planToConfiguration using Eigen vectors.
  virtual trajectory::TrajectoryPtr planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const Eigen::VectorXd &goal,
    const constraint::CollisionFreePtr &collisionFree,
    double timeLimit);

  /// Plans a trajectory to one of the spcified configurations.
  virtual trajectory::TrajectoryPtr planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State *startState,
    const std::vector<statespace::StateSpace::State*> &goalStates,
    const constraint::CollisionFreePtr &collisionFree,
    double timeLimit);

  /// Wrapper for planToConfigurations using Eigen vectors.
  virtual trajectory::TrajectoryPtr planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const std::vector<Eigen::VectorXd> &goals,
    const constraint::CollisionFreePtr &collisionFree,
    double timeLimit);

  /// Plan to a TSR.
  virtual trajectory::TrajectoryPtr planToTSR(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State *startState,
    const constraint::TSR &tsr,
    const constraint::CollisionFreePtr &collisionFree,
    double timelimit);

  /// Wrapper for planToTSR using Eigen vectors.
  virtual trajectory::TrajectoryPtr planToTSR(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const constraint::TSR &tsr,
    const constraint::CollisionFreePtr &collisionFree,
    double timelimit);


  /// Returns a Trajectory that moves the configuration of the metakeleton such
  /// that the specified bodynode is set to a sample in a goal TSR and
  /// the trajectory is constrained to a constraint TSR
  /// \param space The StateSpace for the metaskeleton
  /// \param body Bodynode whose frame is meant for TSR
  /// \param goalTsr The goal TSR to move to
  /// \param constraintTsr The constraint TSR for the trajectory
  /// \param maxNumTrials Max number of trials for solving for IK
  /// \param timelimit Max time (seconds) to spend per planning to each IK
  /// \param collisionFree CollisionFree constraint to check. Self-collision
  /// is checked by default.
  /// \param projectionMaxIteration Parameter for maximum iteration of
  /// constraint projection
  /// \param projectionTolerance Parameter for projection tolerance
  /// \param maxExtensionDistance Parameter for CRRTConnect
  /// \param maxDistanceBtwProjections Parameter for CRRTConnect
  /// \param minStepsize Parameter for CRRTConnect
  /// \param minTreeConnectionDistance Parameter for CRRTConnect
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  virtual trajectory::TrajectoryPtr
  planToTSRwithTrajectoryConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &bodyNode,
      const constraint::TSRPtr &goalTsr,
      const constraint::TSRPtr &constraintTsr,
      int _maxNumTrials,
      double timelimit,
      const constraint::CollisionFreePtr &collisionFree,
      int projectionMaxIteration = 20,
      double projectionTolerance = 1e-4,
      double maxExtensionDistance = std::numeric_limits<double>::infinity(),
      double maxDistanceBtwProjections = 0.1,
      double minStepsize = 0.05,
      double minTreeConnectionDistance = 0.1);

  /// Plan to a named configuration.
  /// \return Trajectory to the configuration, or nullptr if planning fails
  virtual trajectory::TrajectoryPtr planToNamedConfiguration(
    const std::string &name,
    const CollisionFreePtr &collisionFree,
    double timelimit);

  /// Returns a timed trajectory that can be executed by the robot
  virtual trajectory::TrajectoryPtr postprocessPath(
    const trajectory::TrajectoryPtr &path,
    const CollisionFreePtr &collisionFree,
    double timelimit);

  /// Executes a trajectory
  virtual void executeTrajectory(
    const trajectory::TrajectoryPtr &trajectory);

  /// Postprocesses and executes a path
  /// \param timelimit Timelimit for postprocessing.
  virtual void executePath(
    const trajectory::TrajectoryPtr &path,
    const CollisionFreePtr &collisionFree,
    double timelimit);

  /// Set the configuration of the predefined metaskeleton
  /// \param name Name of the predefined metaSkeleton
  /// \param configuration The configuration to set
  /// \throws invalid_argument error if name does not exist.
  void setConfiguration(
    const std::string &name, const Eigen::VectorXd &configuration);

  // Get configuration of the predefined metaskeleton
  /// \throws invalid_argument error if name does not exist.
  Eigen::VectorXd getConfiguration(
    const std::string &name);

  // Get configruation of the whole robot
  Eigen::VectorXd getConfiguration();

  /// \return World containing this robot.
  planner::WorldPtr getParentWorld();

  /// \return Name of this Robot
  std::string getName();

  dart::dynamics::MetaSkeletonPtr getMetaSkeleton();

  /// \return The metaSkeleton and stateSpace pair corresponding to name.
  std::pair<dart::dynamics::dart::dynamics::MetaSkeletonPtr,
      statespace::dart::MetaSkeletonStateSpacePtr>>
  getNamedMetaSkeleton(
    const std::string &name);

  // Simulation step
  virtual void step() = 0;

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

  virtual bool switchControllers(
      const std::vector<std::string>& start_controllers,
      const std::vector<std::string>& stop_controllers);

  constraint::CollisionFreePtr getSelfCollisionConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr &space) const;

  constraint::TestablePtr getTestableCollisionConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const constraint::CollisionFreePtr &collisionFree) const;

  /// Name of this robot
  std::string mName;

  /// World this Robot belongs to. Every robot must belong to a world.
  planner::WorldPtr mWorld;

  dart::dynamics::MetaSkeletonPtr mRobot;
  statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;

  /// Commonly used configurations.
  std::unordered_map<std::string, Configuration> mNamedConfigurations;

  /// Random generator, used for planning and postprocessing
  common::RNGWrapper<std::mt19937> mRng;

  std::shared_ptr<control::TrajectoryExecutor> mTrajectoryExecutor;

  std::unique_ptr<planner::parabolic::ParabolicTimer> mRetimer;

  std::unique_ptr<planner::parabolic::ParabolicSmoother> mSmoother;

  // For trajectory executions.
  std::unique_ptr<common::ExecutorThread> mThread;

  // For ROS
  std::unique_ptr<::ros::NodeHandle> mNode;

  std::unordered_map<std::string,
    std::unique_ptr<::ros::ServiceClient>> mServiceClients;

  std::unique_ptr<control::ros::RosJointStateClient> mJointStateClient;
  std::unique_ptr<common::ExecutorThread> mJointStateThread;

};

using RobotPtr = std::shared_ptr<Robot>;

} // namespace robot
} // namespace aikido
