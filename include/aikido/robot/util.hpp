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
#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {
namespace util {

struct VectorFieldPlannerParameters
{
  VectorFieldPlannerParameters() = default;
  VectorFieldPlannerParameters(
      double linearVelocity,
      double negativeDistanceTolerance,
      double positiveDistanceTolerance,
      double initialStepSize,
      double jointLimitTolerance,
      double constraintCheckResolution,
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
    , projectionTolerance(projectionTolerance)
  {
    // Do nothing
  }

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
/// \param[in] planner Planner to plan with.
/// \param[in] metaSkeleton MetaSkeleton to plan with.
/// \param[in] metaSkeletonStateSpace The StateSpace for the metaskeleton
/// \param[in] goalState Goal state
/// \param[in] constraint Testable constraint to check for collision.
trajectory::TrajectoryPtr planToConfiguration(
    aikido::planner::ConfigurationToConfigurationPlannerPtr planner,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    const aikido::statespace::StateSpace::State* goalState,
    const aikido::constraint::TestablePtr constraint);

/// Parses YAML node for named configurtaions
/// \param[in] node YAML node containing named configurations
/// \return Unordered map of (name, configuration) pairs.
std::unordered_map<std::string, const Eigen::VectorXd>
parseYAMLToNamedConfigurations(const YAML::Node& node);

/// Gets Goal and Constraint TSR for end-effector.
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

/// Get a specific BodyNode of a MetaSkeleton or throw an execption
/// if it doesn't exist
/// \param[in] skeleton MetaSkeleton that should contain the BodyNode
/// \param[in] bodyNodeName Name of the BodyNode we are looking for
/// \return The BodyNode
const dart::dynamics::BodyNode* getBodyNodeOrThrow(
    const dart::dynamics::MetaSkeleton& skeleton,
    const std::string& bodyNodeName);

/// Get a specific BodyNode of a MetaSkeleton or throw an execption
/// if it doesn't exist
/// \param[in] skeleton MetaSkeleton that should contain the BodyNode
/// \param[in] bodyNodeName Name of the BodyNode we are looking for
/// \return The BodyNode
dart::dynamics::BodyNode* getBodyNodeOrThrow(
    dart::dynamics::MetaSkeleton& skeleton, const std::string& bodyNodeName);

} // namespace util
} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_UTIL_HPP_
