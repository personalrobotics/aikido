#ifndef AIKIDO_ROBOT_UTIL_HPP_
#define AIKIDO_ROBOT_UTIL_HPP_

#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/io/yaml.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

// TODO: These are planning methods used in Robot classes. These will be
// removed once we have a Planner API.
namespace util {

struct VectorFieldPlannerParameters
{
  VectorFieldPlannerParameters(
      double linearVelocity = 0.2,
      double negativeDistanceTolerance = 0.001,
      double positiveDistanceTolerance = 0.001,
      double initialStepSize = 0.001,
      double jointLimitTolerance = 1e-3,
      double constraintCheckResolution = 1e-3,
      double linearGain = 1.0,
      double angularGain = 0.2,
      double timestep = 0.1)
    : linearVelocity(linearVelocity)
    , negativeDistanceTolerance(negativeDistanceTolerance)
    , positiveDistanceTolerance(positiveDistanceTolerance)
    , initialStepSize(initialStepSize)
    , jointLimitTolerance(jointLimitTolerance)
    , constraintCheckResolution(constraintCheckResolution)
    , linearGain(linearGain)
    , angularGain(angularGain)
    , timestep(timestep){
          // Do nothing
      };

  double linearVelocity;
  double negativeDistanceTolerance;
  double positiveDistanceTolerance;
  double initialStepSize;
  double jointLimitTolerance;
  double constraintCheckResolution;
  double linearGain;
  double angularGain;
  double timestep;
};

struct CRRTPlannerParameters
{
  CRRTPlannerParameters(
      common::RNG* rng = nullptr,
      std::size_t maxNumTrials = 5,
      double maxExtensionDistance = std::numeric_limits<double>::infinity(),
      double maxDistanceBtwProjections = 0.1,
      double minStepSize = 0.05,
      double minTreeConnectionDistance = 0.1,
      std::size_t projectionMaxIteration = 20,
      double projectionTolerance = 1e-4)
    : rng(rng)
    , maxNumTrials(maxNumTrials)
    , maxExtensionDistance(maxExtensionDistance)
    , maxDistanceBtwProjections(maxDistanceBtwProjections)
    , minStepSize(minStepSize)
    , minTreeConnectionDistance(minTreeConnectionDistance)
    , projectionMaxIteration(projectionMaxIteration)
    , projectionTolerance(projectionTolerance){
          // Do nothing
      };

  common::RNG* rng;
  std::size_t maxNumTrials;
  double maxExtensionDistance;
  double maxDistanceBtwProjections;
  double minStepSize;
  double minTreeConnectionDistance;
  std::size_t projectionMaxIteration;
  double projectionTolerance;
};

/// Plan the robot to a specific configuration.
/// Restores the robot to its initial configuration after planning.
/// \param[in] space The StateSpace for the metaskeleton
/// \param[in] metaSkeleton MetaSkeleton to plan with.
/// \param[in] goalState Goal state
/// \param[in] collisionTestable Testable constraint to check for collision.
/// \param[in] rng Random number generator
/// \param[in] timelimit Max time to spend per planning to each IK
trajectory::InterpolatedPtr planToConfiguration(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const statespace::StateSpace::State* goalState,
    const constraint::TestablePtr& collisionTestable,
    common::RNG* rng,
    double timelimit);

/// Plan the robot to a set of configurations.
/// Restores the robot to its initial configuration after planning.
/// \param[in] space The StateSpace for the metaskeleton
/// \param[in] metaSkeleton MetaSkeleton to plan with.
/// \param[in] goalStates Goal states
/// \param[in] collisionTestable Testable constraint to check for collision.
/// \param[in] rng Random number generator
/// \param[in] timelimit Max time to spend per planning to each IK
trajectory::InterpolatedPtr planToConfigurations(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::vector<statespace::StateSpace::State*>& goalStates,
    const constraint::TestablePtr& collisionTestable,
    common::RNG* rng,
    double timelimit);

/// Plan the configuration of the metakeleton such that
/// the specified bodynode is set to a sample in TSR
/// \param[in] space The StateSpace for the metaskeleton.
/// \param[in] metaSkeleton MetaSkeleton to plan with.
/// \param[in] bodyNode Bodynode whose frame for which TSR is constructed.
/// \param[in] tsr TSR to plan to.
/// \param[in] collisionTestable Testable constraint to check for collision.
/// \param[in] rng Random number generator
/// \param[in] timelimit Max time (seconds) to spend per planning to each IK
/// \param[in] maxNumTrials Number of retries before failure.
/// \return Trajectory to a sample in TSR, or nullptr if planning fails.
trajectory::InterpolatedPtr planToTSR(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const constraint::dart::TSRPtr& tsr,
    const constraint::TestablePtr& collisionTestable,
    common::RNG* rng,
    double timelimit,
    std::size_t maxNumTrials);

/// Returns a Trajectory that moves the configuration of the metakeleton such
/// that the specified bodynode is set to a sample in a goal TSR and
/// the trajectory is constrained to a constraint TSR
/// Uses CRRTPlanner.
/// \param[in] space The StateSpace for the metaskeleton
/// \param[in] metaSkeleton MetaSkeleton to plan with.
/// \param[in] bodyNode Bodynode whose frame is meant for TSR
/// \param[in] goalTsr The goal TSR to move to
/// \param[in] constraintTsr The constraint TSR for the trajectory
/// \param[in] collisionTestable Testable constraint to check for collision.
/// \param[in] timelimit Timelimit for planning
/// \param[in] crrtParameters Parameters to use in planning.
/// \return Trajectory to a sample in TSR, or nullptr if planning fails.
trajectory::InterpolatedPtr planToTSRwithTrajectoryConstraint(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const constraint::dart::TSRPtr& goalTsr,
    const constraint::dart::TSRPtr& constraintTsr,
    const constraint::TestablePtr& collisionTestable,
    double timelimit,
    const CRRTPlannerParameters& crrtParameters = CRRTPlannerParameters());

/// Plan to a desired end-effector offset with fixed orientation.
/// \param[in] space StateSpace for the metaskeleton
/// \param[in] metaSkeleton Metaskeleton to plan with
/// \param[in] body Bodynode for the end effector
/// \param[in] direction Direction unit vector in the world frame
/// \param[in] collisionTestable Collision constraint to check. Self-collision
/// is checked by default.
/// \param[in] distance Distance distance to move, in meters
/// \param[in] timelimit Timelimit for planning
/// \param[in] positionTolerance Tolerance in position
/// \param[in] angularTolerance Tolerance in angle
/// \param[in] vfParameters VectorFieldPlanenr parameters
/// \param[in] crrtParameters CRRTPlanner parameters
/// \return Output trajectory
trajectory::TrajectoryPtr planToEndEffectorOffset(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& body,
    const Eigen::Vector3d& direction,
    const constraint::TestablePtr& collisionTestable,
    double distance,
    double timelimit,
    double positionTolerance = 1e-3,
    double angularTolerance = 1e-3,
    const VectorFieldPlannerParameters& vfParameters
    = VectorFieldPlannerParameters(),
    const CRRTPlannerParameters& crrtParameters = CRRTPlannerParameters());

/// Plan to a desired end-effector offset with fixed orientation using CRRT.
/// \param[in] space StateSpace for the metaskeleton
/// \param[in] metaSkeleton Metaskeleton to plan with
/// \param[in] bodyNode Bodynode for the end effector
/// \param[in] direction Direction unit vector in the world frame
/// \param[in] collisionTestable Collision constraint to check. Self-collision
/// is checked by default.
/// \param[in] distance Distance distance to move, in meters
/// \param[in] timelimit Timelimit for planning
/// \param[in] positionTolerance Tolerance in position
/// \param[in] angularTolerance Tolerance in angle
/// \param[in] crrtParameters CRRTPlanner parameters
/// \return Output trajectory
trajectory::InterpolatedPtr planToEndEffectorOffsetByCRRT(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const constraint::TestablePtr& collisionTestable,
    const Eigen::Vector3d& direction,
    double distance,
    double timelimit,
    double positionTolerance = 1e-3,
    double angularTolerance = 1e-3,
    const CRRTPlannerParameters& crrtParameters = CRRTPlannerParameters());

/// Parses YAML node for named configurtaions
/// \param[in] node YAML node containing named configurations
/// \return Unordered map of (name, configuration) pairs.
std::unordered_map<std::string, const Eigen::VectorXd>
parseYAMLToNamedConfigurations(const YAML::Node& node);

/// Gets Goal and Constraint TSR for End Effector.
/// \param[in] bodyNode End-effector body node
/// \param[in] direction End-effector direction
/// \param[in] distance Offset to move
/// \param[out] goal Goal TSR
/// \param[out] constraint Constraint TSR
/// \param[in] positionTolerance Tolerance in position, used in TSR
/// \param[in] angularTolerance Tolerance in angle, used in TSR
/// \return True if all outputs are successfully made.
bool getGoalAndConstraintTSRForEndEffectorOffset(
    const dart::dynamics::BodyNodePtr& bodyNode,
    const Eigen::Vector3d& direction,
    double distance,
    const constraint::dart::TSRPtr& goal,
    const constraint::dart::TSRPtr& constraint,
    double positionTolerance = 1e-3,
    double angularTolerance = 1e-3);

/// Get a pose at a point looking at another point.
/// \param[in] positionFrom Position to look from
/// \param[in] positionTo Position to look to
/// \return Pose at positionFrom, looking at positionTo.
Eigen::Isometry3d getLookAtIsometry(
    const Eigen::Vector3d& positionFrom, const Eigen::Vector3d& positionTo);

} // namespace util
} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_UTIL_HPP_
