#ifndef AIKIDO_ROBOT_ROBOT_HPP_
#define AIKIDO_ROBOT_ROBOT_HPP_

#include <chrono>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/dynamics/dynamics.hpp>

#include "aikido/common/util.hpp"
#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/control/Executor.hpp"
#include "aikido/control/KinematicSimulationJointCommandExecutor.hpp"
#include "aikido/control/KinematicSimulationTrajectoryExecutor.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/distance/ConfigurationRanker.hpp"
#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/Planner.hpp"
#include "aikido/planner/World.hpp"
#include "aikido/planner/dart/ConfigurationToEndEffectorOffsetPlanner.hpp"
#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Robot)

/// Robot base class for defining basic behaviors common to most robots.
class Robot
{
public:
  ///
  /// Construct a new Robot object.
  ///
  /// \param[in] skeleton The skeleton defining the robot.
  /// \param[in] name The name of the robot.
  /// \param[in] addDefaultExecutors if true, adds the following:
  ///             KinematicSimulationTrajectoryExecutor (active)
  ///             KinematicSimulationPositionExecutor (inactive)
  ///             KinematicSimulationVelocityExecutor (inactive)
  Robot(
      dart::dynamics::SkeletonPtr skeleton,
      const std::string name = "robot",
      bool addDefaultExecutors = true);

  ///
  /// Construct a new Robot as subrobot.
  ///
  /// \param[in] refSkeleton The metaskeleton defining the robot.
  /// \param[in] rootRobot Pointer to parent robot
  /// \param[in] collisionDetector Parent robot's collision detector
  /// \param[in] collisionFilter Parent robot's collision filter
  /// \param[in] name Unique Name of sub-robot
  Robot(
      dart::dynamics::ReferentialSkeletonPtr refSkeleton,
      Robot* rootRobot,
      dart::collision::CollisionDetectorPtr collisionDetector,
      std::shared_ptr<dart::collision::BodyNodeCollisionFilter> collisionFilter,
      const std::string name = "subrobot");

  /// Destructor.
  virtual ~Robot() = default;

  ///
  /// Manually add pairs of body nodes for which collision is ignored
  /// Note: adjacent body pairs are already ignored.
  ///
  /// \param[in] bodyNodeList list of pairs of body node names
  virtual void ignoreSelfCollisionPairs(
      const std::vector<std::pair<std::string, std::string>> bodyNodeList);

  ///
  /// Manually remove pairs of body nodes for which collision constraints
  /// are enforced.
  /// Note: adjacent body pairs start ignored.
  ///
  /// \param[in] bodyNodeList list of pairs of body node names
  virtual void enforceSelfCollisionPairs(
      const std::vector<std::pair<std::string, std::string>> bodyNodeList);

  ///
  /// Steps the robot (and underlying executors and subrobots) through time.
  /// Call regularly to update the state of the robot.
  ///
  /// \param[in] timepoint The point in time to step to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint);

  ///
  /// Get the self-collision constraint of the robot.
  constraint::dart::CollisionFreePtr getSelfCollisionConstraint() const;

  ///
  /// Given a collision constraint, returns it combined with the self-collision
  /// constraint of the entire robot.
  /// \param[in] collisionFree The collision constraint that is to be tied with
  /// root robot's self-collision constraint.
  constraint::TestablePtr combineCollisionConstraint(
      const constraint::dart::CollisionFreePtr& collisionFree) const;

  ///
  /// Get the collision constraint between this (sub)robot
  /// and selected bodies within its World, combined with
  /// its self-collision constraint.
  /// \param[in] bodyNames Names of the bodies in
  /// the robot's world to check collision with.
  /// Leave blank to check with all non-robot bodies.
  constraint::TestablePtr getWorldCollisionConstraint(
      const std::vector<std::string> bodyNames
      = std::vector<std::string>()) const;

  ///
  /// Registers a subset of the joints of the skeleton as a new robot.
  /// Must be disjoint from other subrobots.
  ///
  /// \param[in] metaSkeleton The referential skeleton corresponding to the
  /// subrobot. \param[in] name Name of the subrobot.
  virtual RobotPtr registerSubRobot(
      dart::dynamics::ReferentialSkeletonPtr refSkeleton,
      const std::string& name);

  ///
  /// Get existing subrobot pointer. Or nullptr if not found.
  ///
  /// \param[in] name Name of the subrobot.
  RobotPtr getSubRobotByName(const std::string& name) const;

  /// Returns a named configuration, or size-0 vector if not found.
  /// \param[in] name Name of the configuration.
  Eigen::VectorXd getNamedConfiguration(const std::string& name) const;

  /// Sets the list of named configurations.
  /// \param[in] namedConfigurations Map of name, configuration.
  void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations);

  ///
  /// Plan the robot to a specific configuration.
  ///
  /// \param[in] goalConf Goal configuration
  /// Returns nullptr if not in robot's statespace.
  /// \param[in] testableConstraint Planning (e.g. collision) constraints, set
  /// to nullptr for no constraints (not recommended)
  /// \param[in] planner Configuration planner, defaults to Snap Planner
  trajectory::TrajectoryPtr planToConfiguration(
      const Eigen::VectorXd& goalConf,
      const constraint::TestablePtr& testableConstraint,
      const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
          planner
      = nullptr,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor
      = nullptr) const;

  /// Default to selfCollisionConstraint
  trajectory::TrajectoryPtr planToConfiguration(
      const Eigen::VectorXd& goalConf,
      const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
          planner
      = nullptr,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor
      = nullptr) const;

  ///
  /// Plan a specific body node of the robot to
  /// a position offset.
  ///
  /// \param[in] bodyNodeName Bodynode (usually the end effector) to offset
  /// \param[in] offset Position offset in R^3
  /// \param[in] testableConstraint Planning (e.g. collision) constraints, set
  /// to nullptr for no constraints (not recommended)
  /// \param[in] planner Configuration planner, defaults to VectorFieldPlanner
  /// with robot::util::defaultVFParams
  trajectory::TrajectoryPtr planToOffset(
      const std::string bodyNodeName,
      const Eigen::Vector3d& offset,
      const constraint::TestablePtr& testableConstraint,
      const std::shared_ptr<
          planner::dart::ConfigurationToEndEffectorOffsetPlanner>& planner
      = nullptr,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor
      = nullptr) const;

  /// Default to selfCollisionConstraint
  trajectory::TrajectoryPtr planToOffset(
      const std::string bodyNodeName,
      const Eigen::Vector3d& offset,
      const std::shared_ptr<
          planner::dart::ConfigurationToEndEffectorOffsetPlanner>& planner
      = nullptr,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor
      = nullptr) const;

  ///
  /// Plan a specific body node of the robot to a
  /// sample configuraiton within a Task Space Region.
  ///
  /// \param[in] bodyNodeName Bodynode (usually the end effector) whose frame
  /// should end up in the TSR.
  /// \param[in] tsr \see constraint::dart::TSR
  /// \param[in] testableConstraint Planning (e.g. collision) constraints,
  /// set to nullptr for no constraints (not recommended)
  /// \param[in] maxSamplingTries Maximum number of times for the sample
  /// generator to retry sampling from the TSR.
  /// \param[in] batchSize Number of TSR samples to include per ranked batch.
  /// \param[in] maxBatches Maximum number of ranked batches to run when
  /// planning to TSR samples.
  /// \param[in] planner Base configuration planner, defaults to Snap Planner
  /// \param[in] ranker Ranker to rank the sampled configurations. If nullptr,
  /// NominalConfigurationRanker is used with the current metaSkeleton pose.
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  trajectory::TrajectoryPtr planToTSR(
      const std::string bodyNodeName,
      const constraint::dart::TSRPtr& tsr,
      const constraint::TestablePtr& testableConstraint,
      const util::PlanToTSRParameters& params = util::PlanToTSRParameters(),
      const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
          planner
      = nullptr,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor
      = nullptr,
      const distance::ConstConfigurationRankerPtr& ranker = nullptr) const;

  /// Default to selfCollisionConstraint
  trajectory::TrajectoryPtr planToTSR(
      const std::string bodyNodeName,
      const constraint::dart::TSRPtr& tsr,
      const util::PlanToTSRParameters& params = util::PlanToTSRParameters(),
      const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
          planner
      = nullptr,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor
      = nullptr,
      const distance::ConstConfigurationRankerPtr& ranker = nullptr) const;

  /////////////// Executor Management //////////////////

  ///
  /// Deactivates the active executor.
  /// Clears the inactive executor list.
  /// Forgets all unique id.
  ///
  virtual void clearExecutors();

  ///
  /// Deactivates the active executor.
  ///
  virtual void deactivateExecutor();

  ///
  /// Retrieves the active executor.
  ///
  virtual aikido::control::ExecutorPtr getActiveExecutor();

  ///
  /// Add an executor to the inactive executors list.
  /// Releases DoFs held by executor if held.
  ///
  /// \param[in] executor The Executor to add to the inactive list.
  /// \param[in] desiredName The desired name for the executor.
  /// \return A robot-unique non-empty string ID (empty implies failure)
  virtual std::string registerExecutor(aikido::control::ExecutorPtr executor, std::string desiredName = "");

  ///
  /// Deactivates the current active executor.
  /// Sets an executor from the inactive executor list to be the
  /// active executor.
  /// Holds and releases DoFs as needed.
  ///
  /// \param[in] id of executor on executor list
  /// \return True if successful. If false, all executors are inactive.
  virtual bool activateExecutor(std::string id);

  ///
  /// Convenience:
  /// Deactivates the current active executor.
  /// Activates the *most recently registered* executor
  /// of the given type.
  ///
  /// \param[in] type of executor to activate.
  /// \return True if successful. If false, all executors are inactive.
  virtual bool activateExecutor(const aikido::control::ExecutorType type);

  ///
  /// Convenience: executes given joint command on the robot.
  /// future will have exception if active executor is not
  /// of the appropriate type.
  ///
  /// \param[in] type Type of JointCommandExecutor to look for.
  /// \param[in] command The joint command to execute.
  /// \param[in] timeout Timeout for the joint command
  template <aikido::control::ExecutorType T>
  std::future<int> executeJointCommand(
      const std::vector<double>& command,
      const std::chrono::duration<double>& timeout);

  ///
  /// Convenience: executes given trajectory on the robot.
  /// future will have exception if active executor is not
  /// of type TrajectoryExecutor
  ///
  /// \param[in] trajectory The trajectory to execute.
  std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory);

  ///
  /// Cancels all running commands on this robot.
  ///
  /// \param[in] includeSubrobots Also cancel commands on all subrobots.
  /// \param[in] includeParents Also cancel commands on parent robots
  /// \param[in] excludedSubrobots Ignores theese subrobots when issuing
  /// cancellations
  virtual void cancelAllCommands(
      bool includeSubrobots = true,
      bool includeParents = false,
      const std::vector<std::string> excludeSubrobots
      = std::vector<std::string>());

  /////////////// Getters and Setters //////////////////

  // Gets the (sub)robot's name
  std::string getName() const;

  // Gets the (sub)robot's DoFs
  std::set<std::string> getDofs() const;

  // Gets a copy of the (sub)robot's metaskeleton at the current state
  dart::dynamics::MetaSkeletonPtr getMetaSkeletonClone() const;

  dart::dynamics::MetaSkeletonPtr getMetaSkeleton();

  const dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const;

  // Retrieves current state of the robot
  Eigen::VectorXd getCurrentConfiguration() const;

  // Clones the RNG pointer (or nullptr if not set)
  common::UniqueRNGPtr cloneRNG() const;

  // Sets the RNG pointer
  void setRNG(common::UniqueRNGPtr rng);

  // Gets this robot's (mutable) planning world
  aikido::planner::WorldPtr getWorld() const;

  // Set's this robot's planning world
  // Adds robot to world if not already present
  void setWorld(aikido::planner::WorldPtr world);

  // Sets the default trajectory post-processor.
  // Also enables automatic post-processing for all planning functions.
  void setDefaultPostProcessor(
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor);

  // Get current default postprocessor
  std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
  getDefaultPostProcessor() const;

  // Enables/disables automatic post-processing for all planning functions.
  void setEnablePostProcessing(bool enable = true);

  // Utility function to get root skeleton
  dart::dynamics::ConstSkeletonPtr getRootSkeleton() const;

  // Utility function to get root skeleton
  dart::dynamics::SkeletonPtr getRootSkeleton();

protected:
  std::string mName;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  
  // Currently active executor
  std::string mActiveExecutor{std::string()};
  // Executors indexed by name
  std::unordered_map<std::string, aikido::control::ExecutorPtr> mExecutors;
  // Keeps track of order of addition of Executors
  std::vector<std::string> mExecutorsInsertionOrder;

  // Subrobot and Joint Management
  Robot* mParentRobot{nullptr};
  // Managed degrees of freedom (= mMetaSkeleton->getDofs()->getName())
  std::set<std::string> mDofs;
  // Subrobots indexed by name
  std::unordered_map<std::string, RobotPtr> mSubRobots;

  // Collision objects.
  dart::collision::CollisionDetectorPtr mCollisionDetector;
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;

  // Planning post-processor
  bool mEnablePostProcessing{false};
  std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
      mDefaultPostProcessor{nullptr};

  // Custom random-number-generator for repeatable planning and post-processing
  std::unique_ptr<common::RNG> mRng;

  // Planning world in which the robot resides
  aikido::planner::WorldPtr mWorld{nullptr};

  // Map of commonly-used configurations
  using ConfigurationMap
      = std::unordered_map<std::string, const Eigen::VectorXd>;
  ConfigurationMap mNamedConfigurations;
};

} // namespace robot
} // namespace aikido

#include "detail/Robot-impl.hpp"

#endif // AIKIDO_ROBOT_ROBOT_HPP_
