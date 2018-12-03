#include "aikido/robot/util.hpp"
#include <dart/common/Console.hpp>
#include <dart/common/StlHelpers.hpp>
#include <dart/common/Timer.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/CyclicSampleable.hpp"
#include "aikido/constraint/FiniteSampleable.hpp"
#include "aikido/constraint/NewtonsMethodProjectable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/constraint/dart/FrameDifferentiable.hpp"
#include "aikido/constraint/dart/FrameTestable.hpp"
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/planner/dart/ConfigurationToConfiguration.hpp"
#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToConfiguration.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/planner/ompl/CRRTConnect.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"
#include "aikido/planner/vectorfield/VectorFieldPlanner.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace robot {
namespace util {

using constraint::dart::CollisionFreePtr;
using constraint::dart::InverseKinematicsSampleable;
using constraint::dart::TSR;
using constraint::dart::TSRPtr;
using constraint::TestablePtr;
using constraint::dart::createProjectableBounds;
using constraint::dart::createSampleableBounds;
using constraint::dart::createTestableBounds;
using distance::createDistanceMetric;
using statespace::GeodesicInterpolator;
using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSaver;
using statespace::dart::MetaSkeletonStateSpace;
using statespace::StateSpace;
using trajectory::TrajectoryPtr;
using trajectory::Interpolated;
using trajectory::InterpolatedPtr;
using trajectory::SplinePtr;
using common::cloneRNGFrom;
using common::RNG;
using planner::ConfigurationToConfiguration;
using planner::ConfigurationToConfigurationPlanner;
using planner::ConfigurationToConfigurationPlannerPtr;
using planner::dart::
    ConfigurationToConfiguration_to_ConfigurationToConfiguration;
using dart::collision::FCLCollisionDetector;
using dart::common::make_unique;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::ChainPtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;

static const double collisionResolution = 0.1;

//==============================================================================
trajectory::TrajectoryPtr planToConfiguration(
    ConfigurationToConfigurationPlannerPtr planner,
    const MetaSkeletonPtr& metaSkeleton,
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    const statespace::StateSpace::State* goalState,
    const TestablePtr constraint)
{
  // Get the states
  auto const start = metaSkeletonStateSpace->getScopedStateFromMetaSkeleton(
      metaSkeleton.get());
  auto const goal = metaSkeletonStateSpace->createState();
  metaSkeletonStateSpace->copyState(goalState, goal);

  // Create the problem
  const planner::dart::ConfigurationToConfiguration problem(
      metaSkeletonStateSpace, start, goal, constraint);

  // Convert the planner to a dart planner
  auto dartPlanner = std::
      make_shared<ConfigurationToConfiguration_to_ConfigurationToConfiguration>(
          planner, metaSkeleton);

  // Call plan on the problem
  auto trajectory = dartPlanner->plan(problem);

  if (!trajectory)
    return trajectory;

  return nullptr;
}

//==============================================================================
std::unordered_map<std::string, const Eigen::VectorXd>
parseYAMLToNamedConfigurations(const YAML::Node& node)
{
  std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations;

  for (const auto& configurationNode : node)
  {
    auto configurationName = configurationNode.first.as<std::string>();
    auto configuration = configurationNode.second.as<Eigen::VectorXd>();

    namedConfigurations.emplace(configurationName, configuration);
  }

  return namedConfigurations;
}

//==============================================================================
bool getGoalAndConstraintTSRForEndEffectorOffset(
    const BodyNodePtr& bodyNode,
    const Eigen::Vector3d& direction,
    double distance,
    const TSRPtr& goal,
    const TSRPtr& constraint,
    double positionTolerance,
    double angularTolerance)
{
  if (goal == nullptr || constraint == nullptr)
    return false;

  // create goal TSR
  // 'object frame w' is at end-effector, z pointed along direction to move
  Eigen::Isometry3d H_world_ee = bodyNode->getWorldTransform();
  Eigen::Isometry3d H_world_w
      = getLookAtIsometry(H_world_ee.translation(), direction);
  Eigen::Isometry3d H_w_ee = H_world_w.inverse() * H_world_ee;

  Eigen::Isometry3d Hw_end = Eigen::Isometry3d::Identity();
  Hw_end.translation()[2] = distance;

  goal->mT0_w = H_world_w * Hw_end;
  goal->mTw_e = H_w_ee;
  goal->mBw.setZero();

  // create constraint TSR
  constraint->mT0_w = H_world_w;
  constraint->mTw_e = H_w_ee;
  constraint->mBw << -positionTolerance, positionTolerance, -positionTolerance,
      positionTolerance, 0.0, distance, -angularTolerance, angularTolerance,
      -angularTolerance, angularTolerance, -angularTolerance, angularTolerance;

  return true;
}

//==============================================================================
Eigen::Isometry3d getLookAtIsometry(
    const Eigen::Vector3d& positionFrom, const Eigen::Vector3d& positionTo)
{
  if (positionTo.norm() < 1e-6)
  {
    throw std::runtime_error("positionTo cannot be a zero vector.");
  }
  Eigen::Isometry3d H = Eigen::Isometry3d::Identity();
  H.translation() = positionFrom;

  // original z axis direction
  H.linear()
      = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), positionTo)
            .toRotationMatrix();
  return H;
}

//==============================================================================
const dart::dynamics::BodyNode* getBodyNodeOrThrow(
    const MetaSkeleton& skeleton, const std::string& bodyNodeName)
{
  auto bodyNode = skeleton.getBodyNode(bodyNodeName);

  if (!bodyNode)
  {
    std::stringstream message;
    message << "Bodynode [" << bodyNodeName << "] does not exist in skeleton.";
    throw std::runtime_error(message.str());
  }

  return bodyNode;
}

//==============================================================================
dart::dynamics::BodyNode* getBodyNodeOrThrow(
    MetaSkeleton& skeleton, const std::string& bodyNodeName)
{
  auto bodyNode = skeleton.getBodyNode(bodyNodeName);

  if (!bodyNode)
  {
    std::stringstream message;
    message << "Bodynode [" << bodyNodeName << "] does not exist in skeleton.";
    throw std::runtime_error(message.str());
  }

  return bodyNode;
}

} // namespace util
} // namespace robot
} // namespace aikido
