#include "aikido/robot/util.hpp"
#include <dart/common/Console.hpp>
#include <dart/common/StlHelpers.hpp>
#include <dart/common/Timer.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include "aikido/common/RNG.hpp"
#include <aikido/constraint/CyclicSampleable.hpp>
#include <aikido/constraint/FiniteSampleable.hpp>
#include <aikido/constraint/FrameDifferentiable.hpp>
#include <aikido/constraint/FrameTestable.hpp>
#include <aikido/constraint/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/NewtonsMethodProjectable.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/parabolic/ParabolicSmoother.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

namespace aikido {
namespace robot {
namespace util {

using constraint::CollisionFreePtr;
using constraint::TSR;
using constraint::TSRPtr;
using constraint::TestablePtr;
using constraint::createProjectableBounds;
using constraint::createSampleableBounds;
using constraint::createTestableBounds;
using distance::createDistanceMetric;
using statespace::GeodesicInterpolator;
using statespace::dart::MetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSaver;
using statespace::dart::MetaSkeletonStateSpace;
using statespace::StateSpace;
using trajectory::Interpolated;
using trajectory::InterpolatedPtr;
using trajectory::SplinePtr;
using common::cloneRNGFrom;
using common::RNG;

using dart::collision::FCLCollisionDetector;
using dart::common::make_unique;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::ChainPtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;

static const double collisionResolution = 0.1;
static const size_t maxNumTrials = 3;

//==============================================================================
// TODO: These are temporary methods until we have Planner API.
InterpolatedPtr planToConfiguration(
    const MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const StateSpace::State* goalState,
    const TestablePtr& collisionTestable,
    RNG* rng,
    double timelimit)
{
  using planner::ompl::planOMPL;
  using planner::planSnap;

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // First test with Snap Planner
  planner::PlanningResult pResult;
  InterpolatedPtr untimedTrajectory;

  auto startState = space->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  untimedTrajectory = planSnap(
      space,
      startState,
      goalState,
      std::make_shared<GeodesicInterpolator>(space),
      collisionTestable,
      pResult);

  // Return if the trajectory is non-empty
  if (untimedTrajectory)
    return untimedTrajectory;

  untimedTrajectory = planOMPL<ompl::geometric::RRTConnect>(
      startState,
      goalState,
      space,
      std::make_shared<GeodesicInterpolator>(space),
      createDistanceMetric(space),
      createSampleableBounds(space, rng->clone()),
      collisionTestable,
      createTestableBounds(space),
      createProjectableBounds(space),
      timelimit,
      collisionResolution);

  return untimedTrajectory;
}

//==============================================================================
InterpolatedPtr planToConfigurations(
    const MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const std::vector<StateSpace::State*> goalStates,
    const TestablePtr& collisionTestable,
    RNG* rng,
    double timelimit)
{
  using planner::ompl::planOMPL;

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  auto startState = space->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  for (const auto goalState : goalStates)
  {
    // First test with Snap Planner
    planner::PlanningResult pResult;
    InterpolatedPtr untimedTrajectory;

    untimedTrajectory = planner::planSnap(
        space,
        startState,
        goalState,
        std::make_shared<GeodesicInterpolator>(space),
        collisionTestable,
        pResult);

    // Return if the trajectory is non-empty
    if (untimedTrajectory)
      return untimedTrajectory;

    untimedTrajectory = planOMPL<ompl::geometric::RRTConnect>(
        startState,
        goalState,
        space,
        std::make_shared<GeodesicInterpolator>(space),
        createDistanceMetric(space),
        createSampleableBounds(space, rng->clone()),
        collisionTestable,
        createTestableBounds(space),
        createProjectableBounds(space),
        timelimit,
        collisionResolution);

    return untimedTrajectory;
  }

  return nullptr;
}

//==============================================================================
InterpolatedPtr planToTSR(
    const MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const BodyNodePtr& bn,
    const TSRPtr& tsr,
    const TestablePtr& collisionTestable,
    RNG* rng,
    double timelimit)
{
  using constraint::InverseKinematicsSampleable;

  // Convert TSR constraint into IK constraint
  InverseKinematicsSampleable ikSampleable(
      space,
      metaSkeleton,
      tsr,
      createSampleableBounds(space, rng->clone()),
      InverseKinematics::create(bn),
      maxNumTrials);

  auto generator = ikSampleable.createSampleGenerator();

  // Goal state
  auto goalState = space->createState();

  auto startState = space->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  Eigen::VectorXd goal;

  // TODO: Change this to timelimit once we use a fail-fast planner
  double timelimitPerSample = timelimit / maxNumTrials;

  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // HACK: try lots of snap plans first
  static const std::size_t maxSnapSamples{100};
  std::size_t snapSamples = 0;

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  while (snapSamples < maxSnapSamples && generator->canSample())
  {
    // Sample from TSR
    {
      std::lock_guard<std::mutex> lock(robot->getMutex());
      bool sampled = generator->sample(goalState);
      if (!sampled)
        continue;

      space->convertStateToPositions(goalState, goal);

      // Set to start state
      space->setState(metaSkeleton.get(), startState);
    }
    ++snapSamples;

    planner::PlanningResult pResult;
    auto traj = planner::planSnap(
        space,
        startState,
        goalState,
        std::make_shared<GeodesicInterpolator>(space),
        collisionTestable,
        pResult);

    if (traj)
      return traj;
  }

  // Start the timer
  dart::common::Timer timer;
  timer.start();
  while (timer.getElapsedTime() < timelimit && generator->canSample())
  {
    // Sample from TSR
    {
      std::lock_guard<std::mutex> lock(robot->getMutex());
      bool sampled = generator->sample(goalState);
      if (!sampled)
        continue;

      // Set to start state
      space->setState(metaSkeleton.get(), startState);
    }

    auto traj = planToConfiguration(
        space,
        metaSkeleton,
        goalState,
        collisionTestable,
        rng,
        std::min(timelimitPerSample, timelimit - timer.getElapsedTime()));

    if (traj)
      return traj;
  }

  return nullptr;
}

//==============================================================================
InterpolatedPtr planToTSRwithTrajectoryConstraint(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const constraint::TSRPtr& goalTsr,
    const constraint::TSRPtr& constraintTsr,
    const constraint::TestablePtr& collisionTestable,
    RNG* rng,
    double timelimit,
    int projectionMaxIteration,
    double projectionTolerance,
    double maxExtensionDistance,
    double maxDistanceBtwProjections,
    double minStepsize,
    double minTreeConnectionDistance)
{
  using aikido::constraint::Sampleable;
  using aikido::constraint::InverseKinematicsSampleable;
  using aikido::constraint::CyclicSampleable;
  using aikido::constraint::FrameDifferentiable;
  using aikido::constraint::FrameTestable;
  using aikido::constraint::NewtonsMethodProjectable;
  using aikido::planner::ompl::planCRRTConnect;

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());

  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // Create seed constraint
  std::shared_ptr<Sampleable> seedConstraint
      = std::move(createSampleableBounds(space, rng->clone()));

  // crate IK
  auto ik = InverseKinematics::create(bodyNode);

  // create goal sampleable
  auto goalSampleable = std::make_shared<InverseKinematicsSampleable>(
      space,
      metaSkeleton,
      std::make_shared<CyclicSampleable>(goalTsr),
      seedConstraint,
      ik,
      maxNumTrials);

  // create goal testable
  auto goalTestable = std::make_shared<FrameTestable>(
      space, metaSkeleton, bodyNode.get(), goalTsr);

  // create constraint sampleable
  auto constraintSampleable = std::make_shared<InverseKinematicsSampleable>(
      space, metaSkeleton, constraintTsr, seedConstraint, ik, maxNumTrials);

  // create constraint projectable
  auto frameDiff = std::make_shared<FrameDifferentiable>(
      space, metaSkeleton, bodyNode.get(), constraintTsr);

  std::vector<double> projectionToleranceVec(
      frameDiff->getConstraintDimension(), projectionTolerance);
  auto constraintProjectable = std::make_shared<NewtonsMethodProjectable>(
      frameDiff, projectionToleranceVec, projectionMaxIteration);

  // Current state
  auto startState = space->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  // Call planner
  auto traj = planCRRTConnect(
      startState,
      goalTestable,
      goalSampleable,
      constraintProjectable,
      space,
      std::make_shared<GeodesicInterpolator>(space),
      createDistanceMetric(space),
      constraintSampleable,
      collisionTestable,
      createTestableBounds(space),
      createProjectableBounds(space),
      timelimit,
      maxExtensionDistance,
      maxDistanceBtwProjections,
      minStepsize,
      minTreeConnectionDistance);

  return traj;
}

//==============================================================================
trajectory::SplinePtr planToEndEffectorOffset(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const Eigen::Vector3d& direction,
    const constraint::TestablePtr& collisionTestable,
    double distance,
    double linearVelocity,
    double timelimit,
    double minDistance,
    double maxDistance,
    double positionTolerance,
    double angularTolerance,
    double initialStepSize,
    double jointLimitTolerance,
    double constraintCheckResolution)
{

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());

  auto traj = aikido::planner::vectorfield::planToEndEffectorOffset(
      space,
      metaSkeleton,
      bodyNode,
      collisionTestable,
      direction,
      minDistance,
      maxDistance,
      positionTolerance,
      angularTolerance,
      initialStepSize,
      jointLimitTolerance,
      constraintCheckResolution,
      std::chrono::duration<double>(timelimit));

  return std::move(traj);
}

} // namespace util
} // namespace robot
} // namespace aikido
