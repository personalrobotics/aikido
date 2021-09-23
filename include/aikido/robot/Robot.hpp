#ifndef AIKIDO_ROBOT_ROBOT_HPP_
#define AIKIDO_ROBOT_ROBOT_HPP_

#include <chrono>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/dynamics/dynamics.hpp>

#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/control/KinematicSimulationTrajectoryExecutor.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/distance/ConfigurationRanker.hpp"
#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/Planner.hpp"
#include "aikido/planner/World.hpp"
#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

namespace internal {

inline aikido::control::TrajectoryExecutorPtr trajExecFromSkeleton(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton)
  {
    throw std::invalid_argument("Null MetaskeletonPtr");
  }
  return std::make_shared<
      aikido::control::KinematicSimulationTrajectoryExecutor>(skeleton);
}

} // namespace internal

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
  /// \param[in] trajExecutor Custom executor for trajectories
  Robot(
      dart::dynamics::SkeletonPtr skeleton,
      const std::string name,
      const aikido::control::TrajectoryExecutorPtr trajExecutor);

  ///
  /// Construct a new Robot object.
  ///
  /// \param[in] skeleton The skeleton defining the robot.
  /// \param[in] name The name of the robot.
  Robot(dart::dynamics::SkeletonPtr skeleton, const std::string name = "robot")
    : Robot(skeleton, name, internal::trajExecFromSkeleton(skeleton))
  {
  }

  ///
  /// Construct a new Robot as subrobot.
  ///
  /// \param[in] skeleton The skeleton defining the robot.
  /// \param[in] name The name of the robot.
  Robot(
      dart::dynamics::ReferentialSkeletonPtr refSkeleton,
      RobotPtr rootRobot,
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
  /// Executes given trajectory on the robot.
  /// Note: cancels all running trajectories on root robot(s)
  ///
  /// \param[in] trajectory The trajectory to execute.
  virtual std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) const;

  ///
  /// Cancels all running trajectories on this robot.
  ///
  /// \param[in] includeSubrobots Also cancel trajectories on all subrobots.
  /// \param[in] includeParents Also cancel trajectories on parent robots
  /// \param[in] excludedSubrobots Ignores theese subrobots when issuing
  /// cancellations
  virtual void cancelAllTrajectories(
      bool includeSubrobots = true,
      bool includeParents = false,
      const std::vector<std::string> excludeSubrobots
      = std::vector<std::string>());

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
  /// Get the collission constraint between the root robot
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
      dart::dynamics::ReferentialSkeletonPtr& refSkeleton,
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
      = nullptr) const
  {
    return planToConfiguration(
        goalConf, getSelfCollisionConstraint(), planner, trajPostProcessor);
  }

  ///
  /// Plan a specific body node of the robot to a
  /// sample configuraiton within a Task Space Region.
  ///
  /// \param[in] bodyNodeName Bodynode (usually the end effector) whose frame
  /// should end up in the TSR.
  /// \param[in] tsr \see constraint::dart::TSR
  /// \param[in] testableConstraint Planning (e.g. collision) constraints,
  /// set to nullptr for no constraints (not recommended)
  /// \param[in] maxSamples Maximum number of TSR samples
  /// to plan to (defaults to 1)
  /// \param[in] planner Base configuration planner, defaults to Snap Planner
  /// \param[in] ranker Ranker to rank the sampled configurations. If nullptr,
  /// NominalConfigurationRanker is used with the current metaSkeleton pose.
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  trajectory::TrajectoryPtr planToTSR(
      const std::string bodyNodeName,
      const constraint::dart::TSRPtr& tsr,
      const constraint::TestablePtr& testableConstraint,
      std::size_t maxSamples = 1,
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
      std::size_t maxSamples = 1,
      const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
          planner
      = nullptr,
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor
      = nullptr,
      const distance::ConstConfigurationRankerPtr& ranker = nullptr) const
  {
    return planToTSR(
        bodyNodeName,
        tsr,
        getSelfCollisionConstraint(),
        maxSamples,
        planner,
        trajPostProcessor,
        ranker);
  }

  /////////////// Getters and Setters //////////////////

  // Gets the (sub)robot's name
  std::string getName() const
  {
    return mName;
  }

  // Gets a copy of the (sub)robot's metaskeleton at the current state
  dart::dynamics::MetaSkeletonPtr getMetaSkeletonClone() const
  {
    return mMetaSkeleton->cloneMetaSkeleton();
  }

  dart::dynamics::MetaSkeletonPtr getMetaSkeleton()
  {
    return mMetaSkeleton;
  }

  const dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const
  {
    return mMetaSkeleton;
  }

  // Retrieves current state of the robot
  Eigen::VectorXd getCurrentConfiguration() const
  {
    return mMetaSkeleton->getPositions();
  }

  // Clones the RNG pointer (or nullptr if not set)
  common::UniqueRNGPtr cloneRNG() const
  {
    if (mRng)
    {
      return mRng->clone();
    }
    return nullptr;
  }

  // Sets the RNG pointer
  void setRNG(common::UniqueRNGPtr rng)
  {
    if (rng)
    {
      mRng = std::move(rng);
    }
  }

  // Gets this robot's (mutable) planning world
  aikido::planner::WorldPtr getWorld() const
  {
    if (mParentRobot)
    {
      return mParentRobot->getWorld();
    }
    return mWorld;
  }

  // Set's this robot's planning world
  // Adds robot to world if not already present
  void setWorld(aikido::planner::WorldPtr world)
  {
    mWorld = world;
    if (world)
    {
      auto skeleton = getRootSkeleton();
      if (!world->hasSkeleton(skeleton))
      {
        world->addSkeleton(skeleton);
      }
    }
  }

  // Sets the root robot.
  void setRootRobot(RobotPtr root)
  {
    mParentRobot = root;
  }

  // Sets the Trajectory Executor
  // TODO(egordon) Later: ensure trajectory executor joints match managed DoFs
  void setTrajectoryExecutor(
      const aikido::control::TrajectoryExecutorPtr& trajExecutor)
  {
    if (mTrajectoryExecutor)
    {
      mTrajectoryExecutor->cancel();
    }
    mTrajectoryExecutor = trajExecutor;
  }

  // Sets the default trajectory post-processor.
  // Also enables automatic post-processing for all planning functions.
  void setDefaultPostProcessor(
      const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
          trajPostProcessor)
  {
    mDefaultPostProcessor = trajPostProcessor;
    mEnablePostProcessing = true;
  }

  // Enables/disables automatic post-processing for all planning functions.
  void setEnablePostProcessing(bool enable = true)
  {
    if (enable && !mDefaultPostProcessor)
    {
      // Initialize default post-processor
      mDefaultPostProcessor
          = std::make_shared<aikido::planner::kunzretimer::KunzRetimer>(
              mMetaSkeleton->getVelocityUpperLimits(),
              mMetaSkeleton->getAccelerationUpperLimits());
    }
    mEnablePostProcessing = enable;
  }

  // Utility function to get root skeleton
  dart::dynamics::ConstSkeletonPtr getRootSkeleton() const
  {
    if (mParentRobot)
    {
      return mParentRobot->getRootSkeleton();
    }

    // Root robot should be a real skeleton
    return mMetaSkeleton->getBodyNode(0)->getSkeleton();
  }

  // Utility function to get root skeleton
  dart::dynamics::SkeletonPtr getRootSkeleton()
  {
    if (mParentRobot)
    {
      return mParentRobot->getRootSkeleton();
    }

    // Root robot should be a real skeleton
    return mMetaSkeleton->getBodyNode(0)->getSkeleton();
  }

protected:
  std::string mName;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  aikido::control::TrajectoryExecutorPtr mTrajectoryExecutor{nullptr};

  // Subrobot and Joint Management
  RobotPtr mParentRobot{nullptr};
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

#endif // AIKIDO_ROBOT_ROBOT_HPP_
