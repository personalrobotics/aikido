#include "aikido/robot/util.hpp"

#include <algorithm>

#include <dart/common/Console.hpp>
#include <dart/common/Timer.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "aikido/common/RNG.hpp"
#include "aikido/common/memory.hpp"
#include "aikido/constraint/CyclicSampleable.hpp"
#include "aikido/constraint/FiniteSampleable.hpp"
#include "aikido/constraint/NewtonsMethodProjectable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/constraint/dart/FrameDifferentiable.hpp"
#include "aikido/constraint/dart/FrameTestable.hpp"
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/distance/NominalConfigurationRanker.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/planner/PlanningResult.hpp"
#include "aikido/planner/SnapConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/ompl/CRRTConnect.hpp"
#include "aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/ompl/Planner.hpp"
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

using common::cloneRNGFrom;
using common::RNG;
using constraint::TestablePtr;
using constraint::dart::CollisionFreePtr;
using constraint::dart::createProjectableBounds;
using constraint::dart::createSampleableBounds;
using constraint::dart::createTestableBounds;
using constraint::dart::InverseKinematicsSampleable;
using constraint::dart::TSR;
using constraint::dart::TSRPtr;
using distance::ConstConfigurationRankerPtr;
using distance::createDistanceMetric;
using distance::NominalConfigurationRanker;
using planner::ConfigurationToConfiguration;
using planner::SnapConfigurationToConfigurationPlanner;
using planner::ompl::OMPLConfigurationToConfigurationPlanner;
using statespace::GeodesicInterpolator;
using statespace::StateSpace;
using statespace::dart::MetaSkeletonStateSaver;
using statespace::dart::MetaSkeletonStateSpace;
using statespace::dart::MetaSkeletonStateSpacePtr;
using trajectory::Interpolated;
using trajectory::InterpolatedPtr;
using trajectory::SplinePtr;
using trajectory::TrajectoryPtr;

using dart::collision::FCLCollisionDetector;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::ChainPtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;

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
