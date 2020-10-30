#ifndef AIKIDO_ROBOT_CONCRETEROBOT_HPP_
#define AIKIDO_ROBOT_CONCRETEROBOT_HPP_

#include <chrono>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <dart/dart.hpp>

#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/distance/ConfigurationRanker.hpp"
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
  // Expose base class functions
  using Robot::getMetaSkeleton;
  using Robot::getStateSpace;

  /// Constructor.
  /// \param[in] name Name of the robot.
  /// \param[in] metaSkeleton Metaskeleton of the robot.
  /// \param[in] simulation True for running in simulation.
  /// \param[in] rng Random number generator.
  /// \param[in] trajectoryExecutor Trajectory executor for the metaSkeleton.
  /// \param[in] collisionDetector Collision detector.
  /// \param[in] selfCollisionFilter Collision filter for self collision.
  ConcreteRobot(
      const std::string& name,
      dart::dynamics::MetaSkeletonPtr metaSkeleton,
      bool simulation,
      aikido::common::UniqueRNGPtr rng,
      aikido::control::TrajectoryExecutorPtr trajectoryExecutor,
      dart::collision::CollisionDetectorPtr collisionDetector,
      std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
          selfCollisionFilter);

  virtual ~ConcreteRobot() = default;

  // Documentation inherited.
  virtual std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) const override;

  // Documentation inherited.
  virtual boost::optional<Eigen::VectorXd> getNamedConfiguration(
      const std::string& name) const override;

  // Documentation inherited.
  virtual void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations) override;

  // Documentation inherited.
  virtual std::string getName() const override;

  // Documentation inherited.
  virtual dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const override;

  // Documentation inherited.
  virtual aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
  getStateSpace() const override;

  // Documentation inherited.
  virtual void setRoot(Robot* robot) override;

  // Documentation inherited.
  virtual void step(
      const std::chrono::system_clock::time_point& timepoint) override;

  // Documentation inherited.
  virtual aikido::constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const override;

  // Documentation inherited.
  virtual aikido::constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const constraint::dart::CollisionFreePtr& collisionFree) const override;

  /// Get a postprocessor that respects velocity and acceleration limits. The
  /// specific postprocessor returned is controlled by `postProcessorParams`.
  /// \param[in] metaSkeleton Metaskeleton of the path.
  /// \param[in] postProcessorParams Postprocessor parameters.
  /// \tparam PostProcessor The trajectory postprocessor to use.
  template <typename PostProcessor>
  std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
  getTrajectoryPostProcessor(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const typename PostProcessor::Params& postProcessorParams) const;

  /// Get a postprocessor that respects velocity and acceleration limits. The
  /// specific postprocessor returned is controlled by `postProcessorParams`.
  /// \param[in] velocityLimits Maximum velocity for each dimension.
  /// \param[in] accelerationLimits Maximum acceleration for each dimension.
  /// \param[in] postProcessorParams Postprocessor parameters.
  /// \tparam PostProcessor The trajectory postprocessor to use.
  template <typename PostProcessor>
  std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
  getTrajectoryPostProcessor(
      const Eigen::VectorXd& velocityLimits,
      const Eigen::VectorXd& accelerationLimits,
      const typename PostProcessor::Params& postProcessorParams) const;

  /// Returns a post-processed trajectory that can be executed by the robot.
  /// \param[in] metaSkeleton Metaskeleton of the path.
  /// \param[in] path Geometric path to execute.
  /// \param[in] constraint Must be satisfied after postprocessing. Typically
  /// collision constraint is passed.
  /// \param[in] postProcessorParams Postprocessor parameters.
  /// \tparam PostProcessor The trajectory postprocessor to use.
  template <typename PostProcessor>
  aikido::trajectory::UniqueSplinePtr postProcessPath(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::trajectory::Trajectory* path,
      const constraint::TestablePtr& constraint,
      const typename PostProcessor::Params& postProcessorParams);

  /// Returns a post-processed trajectory that can be executed by the robot.
  /// \param[in] velocityLimits Maximum velocity for each dimension.
  /// \param[in] accelerationLimits Maximum acceleration for each dimension.
  /// \param[in] path Geometric path to execute.
  /// \param[in] constraint Must be satisfied after postprocessing. Typically
  /// collision constraint is passed.
  /// \param[in] postProcessorParams Postprocessor parameters.
  /// \tparam PostProcessor The trajectory postprocessor to use.
  template <typename PostProcessor>
  aikido::trajectory::UniqueSplinePtr postProcessPath(
      const Eigen::VectorXd& velocityLimits,
      const Eigen::VectorXd& accelerationLimits,
      const aikido::trajectory::Trajectory* path,
      const constraint::TestablePtr& constraint,
      const typename PostProcessor::Params& postProcessorParams);

  /// TODO: Replace this with Problem interface.
  /// Plan the robot to a specific configuration. Restores the robot to its
  /// initial configuration after planning.
  /// \param[in] stateSpace The StateSpace for the metaskeleton
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] goalState Goal state
  /// \param[in] collisionFree Testable constraint to check for collision.
  /// \param[in] timelimit Max time to spend per planning to each IK
  /// \return Trajectory. nullptr if fails to find a trajectory.
  aikido::trajectory::TrajectoryPtr planToConfiguration(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::statespace::StateSpace::State* goalState,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// TODO: Replace this with Problem interface.
  /// Wrapper for planToConfiguration using Eigen vectors.
  /// Plan the robot to a specific configuration. Restores the robot to its
  /// initial configuration after planning.
  /// \param[in] stateSpace The StateSpace for the metaskeleton
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] goal Goal position
  /// \param[in] collisionFree Testable constraint to check for collision.
  /// \param[in] timelimit Max time to spend per planning to each IK
  /// \return Trajectory. nullptr if fails to find a trajectory.
  aikido::trajectory::TrajectoryPtr planToConfiguration(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const Eigen::VectorXd& goal,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// TODO: Replace this with Problem interface.
  /// Plan the robot to a set of configurations. Restores the robot to its
  /// initial configuration after planning.
  /// \param[in] stateSpace The StateSpace for the metaskeleton
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] goalStates A set of goal states.
  /// \param[in] collisionFree Testable constraint to check for collision.
  /// \param[in] timelimit Max time to spend per planning to each IK
  /// \return Trajectory. nullptr if fails to find a trajectory.
  aikido::trajectory::TrajectoryPtr planToConfigurations(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::vector<aikido::statespace::StateSpace::State*>& goalStates,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// TODO: Replace this with Problem interface.
  /// Plan the robot to a set of configurations. Restores the robot to its
  /// initial configuration after planning.
  /// \param[in] stateSpace The StateSpace for the metaskeleton
  /// \param[in] metaSkeleton Metaskeleton to plan with.
  /// \param[in] goals A set of goals.
  /// \param[in] collisionFree Testable constraint to check for collision.
  /// \param[in] timelimit Max time to spend per planning to each IK
  /// \return Trajectory. nullptr if fails to find a trajectory.
  aikido::trajectory::TrajectoryPtr planToConfigurations(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::vector<Eigen::VectorXd>& goals,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// TODO: Replace this with Problem interface.
  /// Plans the configuration of the metakeleton such that
  /// the specified bodynode is set to a sample in TSR
  /// \param[in] stateSpace The StateSpace for the metaskeleton.
  /// \param[in] metaSkeleton The MetaSkeleton to plan with.
  /// \param[in] bodyNode Bodynode whose frame for which TSR is constructed.
  /// \param[in] tsr TSR
  /// \param[in] collisionFree Testable constraint to check for collision.
  /// \param[in] timelimit Max time (seconds) to spend per planning to each IK
  /// \param[in] maxNumTrials Max numer of trials to plan.
  /// \param[in] ranker Ranker to rank the sampled configurations. If nullptr,
  /// NominalConfigurationRanker is used with the current metaSkeleton pose.
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  aikido::trajectory::TrajectoryPtr planToTSR(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& bodyNode,
      const aikido::constraint::dart::TSRPtr& tsr,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit,
      std::size_t maxNumTrials,
      const distance::ConstConfigurationRankerPtr& ranker = nullptr);

  /// Plans to a named configuration.
  /// \param[in] name Name of the configuration to plan to
  /// \param[in] collisionFree Collision constraint
  /// \param[in] timelimit Max time (seconds) to spend
  /// \return Trajectory to the configuration, or nullptr if planning fails
  aikido::trajectory::TrajectoryPtr planToNamedConfiguration(
      const std::string& name,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

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

  std::unique_ptr<aikido::common::RNG> cloneRNG();

  /// If this robot belongs to another (Composite)Robot,
  /// mRootRobot is the topmost robot containing this robot.
  Robot* mRootRobot;

  /// Name of this robot
  std::string mName;

  /// MetaSkeleton of this robot
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;

  /// Skeleton containing mRobot
  dart::dynamics::SkeletonPtr mParentSkeleton;

  /// True if running in simulation mode
  // bool mSimulation;
  // TODO(JS): Disabled since not used in this class

  std::unique_ptr<common::RNG> mRng;

  std::shared_ptr<control::TrajectoryExecutor> mTrajectoryExecutor;

  // double mCollisionResolution;
  // TODO(JS): Disabled since not used in this class

  /// Commonly used configurations.
  ConfigurationMap mNamedConfigurations;

  ::dart::collision::CollisionDetectorPtr mCollisionDetector;
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;
};

} // namespace robot
} // namespace aikido

#include "detail/ConcreteRobot-impl.hpp"

#endif // AIKIDO_ROBOT_CONCRETEROBOT_HPP_
