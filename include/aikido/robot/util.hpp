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
#include "aikido/distance/ConfigurationRanker.hpp"
#include "aikido/io/yaml.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

// TODO: These are mostly planning methods used in Robot classes.
// The planning methods will be removed once we have a Planner API.
namespace util {

/// Default Vector Field Planner Parameters
/// Suitable for ADA-robot end-effector offsets
const std::vector<double> defaultVFParams{
    0.03,  // distanceTolerance
    0.004, // positionTolerance
    0.004, // angularTolerance
    0.01,  // initialStepSize
    1e-3,  // jointlimitTolerance
    1e-3,  // constraintCheckResolution
    1.0};  // timeout

/// Paramters for Vector Field Planner
struct VectorFieldPlannerParameters
{
  /// Struct Constructor
  /// \param[in] distanceTolerance (m) How much a planned trajectory is allowed
  /// to deviate from the requested distance to move the body node \param[in]
  /// positionTolerance (m) How a planned trajectory is allowed to deviated from
  /// a straight line segment defined by the offset \param[in] angularTolerance
  /// (rad) How a planned trajectory is allowed to deviate from a given offset
  /// direction. \param[in] initialStepSize (m) Initial step size. \param[in]
  /// jointLimitTolerance (defined by joint) If less then this distance to joint
  /// limit, velocity is bounded in that direction to 0.
  /// \param[in] constraintCheckResolution Resolution used in constraint
  /// checking.
  /// \param[in] timeout timeout in seconds.
  VectorFieldPlannerParameters(
      double distanceTolerance = defaultVFParams[0],
      double positionTolerance = defaultVFParams[1],
      double angularTolerance = defaultVFParams[2],
      double initialStepSize = defaultVFParams[3],
      double jointLimitTolerance = defaultVFParams[4],
      double constraintCheckResolution = defaultVFParams[5],
      std::chrono::duration<double> timeout
      = std::chrono::duration<double>(defaultVFParams[6]))
    : distanceTolerance(distanceTolerance)
    , positionTolerance(positionTolerance)
    , angularTolerance(angularTolerance)
    , initialStepSize(initialStepSize)
    , jointLimitTolerance(jointLimitTolerance)
    , constraintCheckResolution(constraintCheckResolution)
    , timeout(timeout){
          // Do nothing
      };

  double distanceTolerance;
  double positionTolerance;
  double angularTolerance;
  double initialStepSize;
  double jointLimitTolerance;
  double constraintCheckResolution;
  std::chrono::duration<double> timeout;
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

struct PlanToTSRParameters
{
  PlanToTSRParameters(
    std::size_t maxSamplingTries = 1,
    std::size_t batchSize = 100,
    std::size_t maxBatches = 1)
    : maxSamplingTries(maxSamplingTries)
    , batchSize(batchSize)
    , maxBatches(maxBatches)
  {
    // Do nothing
  }

  std::size_t maxSamplingTries;
  std::size_t batchSize;
  std::size_t maxBatches;
};

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

/// Get a set of degree-of-freedom names from MetaSkeleton
/// \param[in] skeleton MetaSkeleton pointer
/// \return A set of DoFs with skeleton->getNumDofs() elements
inline std::set<std::string> dofNamesFromSkeleton(
    const dart::dynamics::MetaSkeletonPtr& skeleton)
{
  std::set<std::string> ret;
  if (!skeleton)
    return ret;
  for (const auto& dof : skeleton->getDofs())
  {
    ret.insert(dof->getName());
  }
  return ret;
}

} // namespace util
} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_UTIL_HPP_
