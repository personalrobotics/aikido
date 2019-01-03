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
#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"
#include "aikido/robot/Robot.hpp"
#include "aikido/robot/util.hpp"
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
  virtual aikido::trajectory::UniqueSplinePtr smoothPath(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::trajectory::Trajectory* path,
      const constraint::TestablePtr& constraint) override;

  // Documentation inherited.
  virtual aikido::trajectory::UniqueSplinePtr retimePath(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::trajectory::Trajectory* path) override;

  // Documentation inherited.
  virtual aikido::trajectory::UniqueSplinePtr retimePathWithKunzTimer(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const aikido::trajectory::Trajectory* path,
      double maxDeviation, double timestep) override;

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

  // Get a smoothing post postprocessor that respects velocity and acceleration
  // limits, as well as the passed constraint.
  /// \param[in] metaSkeleton The MetaSkeleton whose limits must be respected.
  std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
  getTrajectoryPostProcessor(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      bool enableShortcut,
      bool enableBlend,
      double shortcutTimelimit,
      double blendRadius,
      int blendIterations,
      double feasibilityCheckResolution,
      double feasibilityApproxTolerance) const;

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
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  aikido::trajectory::TrajectoryPtr planToTSR(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& bodyNode,
      const aikido::constraint::dart::TSRPtr& tsr,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit,
      std::size_t maxNumTrials);

  /// TODO: Replace this with Problem interface.
  /// Returns a Trajectory that moves the configuration of the metakeleton such
  /// that the specified bodynode is set to a sample in a goal TSR and
  /// the trajectory is constrained to a constraint TSR
  /// \param[in] stateSpace The StateSpace for the metaskeleton
  /// \param[in] metaSkeleton The MetaSkeleton to plan with.
  /// \param[in] bodyNode Bodynode whose frame is meant for TSR
  /// \param[in] goalTsr The goal TSR to move to
  /// \param[in] constraintTsr The constraint TSR for the trajectory
  /// \param[in] collisionFree Collision constraint
  /// \param[in] timelimit Max time (seconds) to spend per planning to each IK
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  aikido::trajectory::TrajectoryPtr planToTSRwithTrajectoryConstraint(
      const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const dart::dynamics::BodyNodePtr& bodyNode,
      const aikido::constraint::dart::TSRPtr& goalTsr,
      const aikido::constraint::dart::TSRPtr& constraintTsr,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// Plans to a named configuration.
  /// \param[in] name Name of the configuration to plan to
  /// \param[in] collisionFree Collision constraint
  /// \param[in] timelimit Max time (seconds) to spend
  /// \return Trajectory to the configuration, or nullptr if planning fails
  aikido::trajectory::TrajectoryPtr planToNamedConfiguration(
      const std::string& name,
      const aikido::constraint::dart::CollisionFreePtr& collisionFree,
      double timelimit);

  /// TODO: This should be revisited once we have Planner API.
  /// Sets CRRTPlanner parameters.
  /// \param[in] crrtParameters CRRT planner parameters
  void setCRRTPlannerParameters(
      const util::CRRTPlannerParameters& crrtParameters);

private:
  // Named Configurations are read from a YAML file
  using ConfigurationMap
      = std::unordered_map<std::string, const Eigen::VectorXd>;

  std::unique_ptr<aikido::common::RNG> cloneRNG();

  /// Compute velocity limits from the MetaSkeleton
  Eigen::VectorXd getVelocityLimits(
      const dart::dynamics::MetaSkeleton& metaSkeleton) const;

  /// Compute acceleration limits from the MetaSkeleton
  Eigen::VectorXd getAccelerationLimits(
      const dart::dynamics::MetaSkeleton& metaSkeleton) const;

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

  util::CRRTPlannerParameters mCRRTParameters;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_CONCRETEROBOT_HPP_
