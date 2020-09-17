#ifndef AIKIDO_ROBOT_ROBOT_HPP_
#define AIKIDO_ROBOT_ROBOT_HPP_

#include <chrono>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <dart/dynamics/dynamics.hpp>

#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/distance/ConfigurationRanker.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"
#include "aikido/robot/Robot.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Robot)

/// Robot interface for defining basic behaviors of a robot.
class Robot
{
public:
  /// Constructor.
  /// \param[in] name Name of the robot.
  /// \param[in] metaSkeleton Metaskeleton of the robot.
  /// \param[in] rng Random number generator.
  /// \param[in] trajectoryExecutor Trajectory executor for the metaSkeleton.
  /// \param[in] collisionDetector Collision detector.
  /// \param[in] selfCollisionFilter Collision filter for self collision.
  // TODO(avk): Why should trajectory executor be passed in? Contruct in place?
  // TODO(avk): Why should collision detector be passed in? Construct in place?
  // TODO(avk): Why should self collision filter be passed in? Get from SRDF?
  // TODO(avk): Why should rng be passed in? What is it even used for?
  // TODO(avk): Why should metaskeleton be passed in? Construct from
  Robot(
      const std::string& name,
      dart::dynamics::MetaSkeletonPtr metaSkeleton,
      aikido::common::UniqueRNGPtr rng,
      aikido::control::TrajectoryExecutorPtr trajectoryExecutor,
      dart::collision::CollisionDetectorPtr collisionDetector,
      std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
          selfCollisionFilter);

  /// Destructor.
  virtual ~Robot() = default;

  /// \return Name of this robot.
  virtual std::string getName() const;

  /// \return [const] MetaSkeleton of this robot.
  virtual dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const;

  /// \return MetaSkeleton of this robot.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton();

  /// \return [const] MetaSkeletonStateSpace of this robot.
  virtual aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
  getStateSpace() const;

  /// \return MetaSkeletonStateSpace of this robot.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr getStateSpace();

  /// Returns a named configuration.
  /// \param[in] name Name of the configuration.
  /// \return Configuration.
  virtual boost::optional<Eigen::VectorXd> getNamedConfiguration(
      const std::string& name) const;

  /// Sets the list of named configurations.
  /// \param[in] namedConfigurations Map of name, configuration.
  virtual void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations);

  /// Sets the root of this robot. If this robot instance is a part of another
  /// robot object, the root allows computation of the full collision
  /// constraint.
  /// \param[in] robot Root robot.
  virtual void setRoot(Robot* robot);

  /// Executes a trajectory.
  /// \param[in] trajectory Timed trajectory to execute.
  virtual std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) const;

  /// Simulates up to the provided timepoint.
  /// \note Assumes that the robot's skeleton is locked.
  /// \param[in] timepoint Time to simulate to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint);

  /// Returns self-collision constraint.
  /// \param[in] space Space in which this collision constraint operates.
  /// \param[in] metaSkeleton Metaskeleton for the statespace.
  virtual constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const;

  /// TODO: Consider changing this to return a CollisionFreePtr.
  /// Combines provided collision constraint with self collision
  /// constraint.
  /// \param[in] space Space in which this collision constraint operates.
  /// \param[in] metaSkeleton Metaskeleton for the statespace.
  /// \param[in] collisionFree Collision constraint.
  /// \return Combined constraint.
  virtual constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const constraint::dart::CollisionFreePtr& collisionFree) const;

  /// Computes velocity limits from the MetaSkeleton. These should be
  /// interpreted as absolute in both the positive and negative directions.
  /// \param[in] metaSkeleton MetaSkeleton to compute limits for.
  /// \return Symmetric velocity limits.
  Eigen::VectorXd getVelocityLimits(
      const dart::dynamics::MetaSkeleton& metaSkeleton) const;

  /// Computes acceleration limits from the MetaSkeleton. These should be
  /// interpreted as absolute in both the positive and negative directions.
  /// \param[in] metaSkeleton MetaSkeleton to compute limits for.
  /// \return Symmetric acceleration limits.
  Eigen::VectorXd getAccelerationLimits(
      const dart::dynamics::MetaSkeleton& metaSkeleton) const;

private:
  // Named Configurations are read from a YAML file
  using ConfigurationMap
      = std::unordered_map<std::string, const Eigen::VectorXd>;

  /// Clones the associated random number generator.
  /// \return a cloned copy of the associated RNG.
  std::unique_ptr<aikido::common::RNG> cloneRNG();

  /// If this robot belongs to another (Composite)Robot,
  /// mRootRobot is the topmost robot containing this robot.
  Robot* mRootRobot;

  /// Name of this robot.
  std::string mName;

  /// MetaSkeleton of this robot.
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  /// The statespace of this robot.
  statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;

  /// Skeleton containing mRobot.
  dart::dynamics::SkeletonPtr mParentSkeleton;

  /// RNG.
  std::unique_ptr<common::RNG> mRng;

  /// Trajectory executor.
  std::shared_ptr<control::TrajectoryExecutor> mTrajectoryExecutor;

  /// Collision detector to evaluate collisions.
  ::dart::collision::CollisionDetectorPtr mCollisionDetector;

  /// Collision filter to ignore collisions between adjacent bodies.
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;

  /// Commonly used configurations that the robot can be in.
  ConfigurationMap mNamedConfigurations;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_ROBOT_HPP_
