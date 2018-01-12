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
#include <aikido/planner/Planner.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Trajectory.hpp>
#include <aikido/planner/postprocessor/Retimer.hpp>
#include <aikido/planner/postprocessor/Smoother.hpp>

namespace aikido {
namespace robot {

/// A base class for robots.
/// Robot only works on MetaSkeletonStateSpace.
/// It is expected that each robot inherits from this.
class Robot
{
public:

  struct Configuration
  {
    const statespace::MetaSkeletonStateSpacePtr metaSkeletonStateSpace;
    const dart::dynamics::MetaSkeletonPtr metaSkeleton;
    const Eigen::VectorXd positions;
  }

  /// Clones this Robot.
  /// \param newName New name for this robot
  virtual std::unique_ptr<Robot> clone(const std::string& newName) = 0;

  virtual aikido::trajectory::TrajectoryPtr planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State *startState,
    const statespace::StateSpace::State *goalState,
    const CollisionFreePtr& collisionFree,
    double timeLimit);

  /// Wrapper for planToConfiguration using Eigen vectors.
  virtual aikido::trajectory::TrajectoryPtr planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const Eigen::VectorXd &goal,
    const CollisionFreePtr &collisionFree,
    double timeLimit);

  /// Wrapper for planToConfiguration using Cnofiguration.
  virtual aikido::trajectory::TrajectoryPtr planToConfiguration(
    const Configuration &configuration,
    const CollisionFreePtr &collisionFree,
    double timeLimit);

  virtual aikido::trajectory::TrajectoryPtr planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State *startState,
    const std::vector<statespace::StateSpace::State*> &goalStates,
    const CollisionFreePtr &collisionFree,
    double timeLimit);

  /// Wrapper for planToConfigurations using Eigen vectors.
  virtual aikido::trajectory::TrajectoryPtr planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const std::vector<Eigen::VectorXd> &goals,
    const CollisionFreePtr &collisionFree,
    double timeLimit);

  virtual aikido::trajectory::TrajectoryPtr planToTSR(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State *startState,
    const aikido::constraint::TSR &tsr,
    const CollisionFreePtr &collisionFree,
    double timelimit);

  /// Wrapper for planToTSR using Eigen vectors.
  virtual aikido::trajectory::TrajectoryPtr planToTSR(
    const statespace::MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const aikido::constraint::TSR &tsr,
    const CollisionFreePtr &collisionFree,
    double timelimit);

  virtual aikido::trajectory::TrajectoryPtr planToNamedConfiguration(
    const std::string &name,
    const CollisionFreePtr &collisionFree,
    double timelimit);

  /// Returns a timed trajectory that can be executed by the robot
  virtual aikido::trajectory::TrajectoryPtr postprocessPath(
    const aikido::trajectory::TrajectoryPtr &path,
    const CollisionFreePtr &collisionFree,
    double timelimit);

  /// Executes a trajectory
  virtual void executeTrajectory(
    const aikido::trajectory::TrajectoryPtr &trajectory);

  /// Postprocesses and executes a path
  /// \param timelimit Timelimit for postprocessing.
  virtual void executePath(
    const aikido::trajectory::TrajectoryPtr &path,
    const CollisionFreePtr &collisionFree,
    double timelimit);

  /// Set the configuration of the predefined metaskeleton
  /// \param name Name of the predefined metaSkeleton
  /// \param configuration The configuration to set
  /// \throws invalid_argument error if name does not exist.
  void setConfiguration(
    const std::string& name, const Eigen::VectorXd& configuration);

  // Get configuration of the predefined metaskeleton
  /// \throws invalid_argument error if name does not exist.
  Eigen::VectorXd getConfiguration(
    const std::string& name);

  // Get configruation of the whole robot
  Eigen::VectorXd getConfiguration();

  /// \return World containing this robot.
  aikido::planner::WorldPtr getParentWorld();

  /// \return Name of this Robot
  std::string getName();

  dart::dynamics::SkeletonPtr getRobot();

  /// Returns the metaSkeleton and stateSpace pair corresponding to name.
  std::pair<dart::dynamics::MetaSkeletonPtr,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr>>
  getMetaSkeletonStateSpacePair(
    const std::string& name);

protected:

  /// Returns true if this Robot contains this metaSkeleton.
  virtual bool checkIfMetaSkeletonBelongs(
    const MetaSkeletonPtr &metaSkeleton);

  virtual bool switchControllers(
      const std::vector<std::string>& start_controllers,
      const std::vector<std::string>& stop_controllers);

  // Simulation step
  virtual void step() = 0;

  aikido::constraint::CollisionFreePtr getSelfCollisionConstraint(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr &space) const;

  aikido::constraint::TestablePtr getTestableCollisionConstraint(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr &space,
      const aikido::constraint::CollisionFreePtr &collisionFree) const;


  /// Name of this robot
  std::string mName;

  double mSmootherFeasibilityCheckResolution;
  double mSmootherFeasibilityApproxTolerance;

  /// MetaPlanner
  std::unique_ptr<aikido::planner::Planner> mPlanner;

  /// World this Robot belongs to. Every robot must be tied to one world.
  aikido::planner::WorldPtr mWorld;

  dart::dynamics::SkeletonPtr mRobot;

  /// Set of commonly used metaskeletons and statespaces maintained by this Robot.
  std::unordered_map<std::string,
    std::pair<dart::dynamics::MetaSkeletonPtr,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr>> mMetaSkeletons;

  /// Named configurations.
  std::unordered_map<std::string, Configuration> mNamedConfigurations;

  /// Random generator, used for planning and postprocessing
  aikido::common::RNGWrapper<std::mt19937> mRng;

  std::shared_ptr<aikido::control::TrajectoryExecutor> mTrajectoryExecutor;

  std::unique_ptr<aikido::planner::postprocessor::Retimer> mRetimer;

  std::unique_ptr<aikido::planner::postprocessor::Smoother> mSmoother;

  // For trajectory executions.
  std::unique_ptr<aikido::common::ExecutorThread> mThread;

  // For ROS
  std::unique_ptr<::ros::NodeHandle> mNode;

  std::unordered_map<std::string,
    std::unique_ptr<::ros::ServiceClient>> mServiceClients;

  std::unique_ptr<aikido::control::ros::RosJointStateClient> mJointStateClient;
  std::unique_ptr<aikido::common::ExecutorThread> mJointStateThread;

}

using RobotPtr = std::shared_ptr<Robot>;

} // namespace robot
} // namespace aikido
