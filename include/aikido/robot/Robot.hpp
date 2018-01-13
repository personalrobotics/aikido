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

  /// Create a new Robot
  /// \param name Name for the new Robot
  virtual static std::unique_ptr<Robot> create(const std::string &name) = 0;

  /// Clones this Robot.
  /// \param newName New name for this robot
  virtual std::unique_ptr<Robot> clone(const std::string& newName) = 0;

  virtual aikido::trajectory::TrajectoryPtr planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const statespace::StateSpace::State *startState,
    const statespace::StateSpace::State *goalState,
    const CollisionFreePtr& collisionFree,
    double timeLimit);

  /// Wrapper for planToConfiguration using Eigen vectors.
  virtual aikido::trajectory::TrajectoryPtr planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const Eigen::VectorXd &start,
    const Eigen::VectorXd &goal,
    const CollisionFreePtr &collisionFree,
    double timeLimit);

  virtual aikido::trajectory::TrajectoryPtr planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const statespace::StateSpace::State *startState,
    const std::vector<statespace::StateSpace::State*> &goalStates,
    const CollisionFreePtr &collisionFree,
    double timeLimit);

  /// Wrapper for planToConfigurations using Eigen vectors.
  virtual aikido::trajectory::TrajectoryPtr planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const Eigen::VectorXd &start,
    const std::vector<Eigen::VectorXd> &goals,
    const CollisionFreePtr &collisionFree,
    double timeLimit);

  virtual aikido::trajectory::TrajectoryPtr planToTSR(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const statespace::StateSpace::State *startState,
    const aikido::constraint::TSR &tsr,
    const CollisionFreePtr &collisionFree,
    double timelimit);

  /// Wrapper for planToTSR using Eigen vectors.
  virtual aikido::trajectory::TrajectoryPtr planToTSR(
    const statespace::MetaSkeletonStateSpacePtr& stateSpace,
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
  virtual void executePath(
    const aikido::trajectory::TrajectoryPtr &path,
    const CollisionFreePtr &collisionFree);

  /// \return World containing this robot.
  aikido::planner::WorldPtr getParentWorld();

  /// \return Name of this Robot
  std::string getName();

protected:

  struct Configuration
  {
    const MetaSkeletonStateSpacePtr metaSkeletonStateSpace;
    const Eigen::VectorXd configuration;
  };

  /// Returns true if this Robot contains
  /// a skeleton with matching statespace
  virtual bool checkIfStateSpaceBelongs(
    const statespace::MetaSkeletonStateSpacePtr& stateSpace);

  bool switchControllers(
      const std::vector<std::string>& start_controllers,
      const std::vector<std::string>& stop_controllers);

  // Simulation step
  void step();

  /// Name of this robot
  std::string mName;


  double mSmootherFeasibilityCheckResolution;
  double mSmootherFeasibilityApproxTolerance;

  // For trajectory executions.
  std::unique_ptr<aikido::common::ExecutorThread> mThread;

  /// MetaPlanner
  std::unique_ptr<aikido::planner::Planner> mPlanner;

  /// World this Robot belongs to. Every robot must be tied to one world.
  aikido::planner::WorldPtr mWorld;

  /// List of meta skeletons maintained by this Robot.
  std::vector<const dart::dynamics::MetaSkeletonPtr> mSkeletons;

  /// List of statespaces maintained by this Robot.
  /// This has one-to-one mapping with mSkeletons.
  std::vector<
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr> mStateSpaces;

  /// Named configurations.
  std::unordered_map<std::string, Configuration> mNamedConfigurations;

  /// Random generator, used for planning and postprocessing
  aikido::common::RNGWrapper<std::mt19937> mRng;

  std::shared_ptr<aikido::control::TrajectoryExecutor> mTrajectoryExecutor;

  std::unique_ptr<aikido::planner::postprocessor::Retimer> mRetimer;

  std::unique_ptr<aikido::planner::postprocessor::Smoother> mSmoother;

};

} // namespace robot
} // namespace aikido
