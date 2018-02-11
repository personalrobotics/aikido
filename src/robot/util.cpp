#include "aikido/robot/util.hpp"

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
using statespace::dart::MetaSkeletonStateSpaceSaver;
using statespace::StateSpace::State;
using trajectory::Interpolated;
using trajectory::InterpolatedPtr;
using common::cloneRNGFrom;

using dart::collision::FCLCollisionDetector;
using dart::common::make_unique;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::ChainPtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;


// TODO: These are temporary methods until we have Planner API.
InterpolatedPtr
planToConfiguration(const MetaSkeletonStateSpacePtr &space,
                    const MetaSkeletonPtr &metaSkeleton,
                    const State* startState,
                    const State* goalState,
                    double timelimit,
                    const TestablePtr &collisionTestable, RNG *rng,
                    double collisionResolution) {
  using planner::ompl::planOMPL;
  using planner::planSnap;

  auto robot = metaSkeleton->getBodyNode(0)->getRobot();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // First test with Snap Planner
  planner::PlanningResult pResult;
  InterpolatedPtr untimedTrajectory;

  untimedTrajectory =
      planSnap(space, startState, goalState,
                                std::make_shared<GeodesicInterpolator>(space),
                                collisionTestable, pResult);

  // Return if the trajectory is non-empty
  if (untimedTrajectory)
    return untimedTrajectory;

  untimedTrajectory = planOMPL<ompl::geometric::RRTConnect>(
      startState, goalState, space,
      std::make_shared<GeodesicInterpolator>(space),
      createDistanceMetric(space),
      createSampleableBounds(space, rng->clone()), collisionTestable,
      createTestableBounds(space), createProjectableBounds(space), timelimit,
      collisionResolution);

  return untimedTrajectory;
}

//==============================================================================
InterpolatedPtr
planToConfigurations(const MetaSkeletonStateSpacePtr &space,
                    const MetaSkeletonPtr &metaSkeleton,
                    const State* startState,
                    const std::vector<State*> goalStates,
                    double timelimit,
                    const TestablePtr &collisionTestable, RNG *rng,
                    double collisionResolution) {
  using planner::ompl::planOMPL;

  auto robot = metaSkeleton->getBodyNode(0)->getRobot();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  for (const auto goalState: goalStates)
  {
    // First test with Snap Planner
    planner::PlanningResult pResult;
    InterpolatedPtr untimedTrajectory;

    untimedTrajectory =
        planner::planSnap(space, startState, goalState,
                                  std::make_shared<GeodesicInterpolator>(space),
                                  collisionTestable, pResult);

    // Return if the trajectory is non-empty
    if (untimedTrajectory)
      return untimedTrajectory;

    untimedTrajectory = planOMPL<ompl::geometric::RRTConnect>(
        startState, goalState, space,
        std::make_shared<GeodesicInterpolator>(space),
        createDistanceMetric(space),
        createSampleableBounds(space, rng->clone()), collisionTestable,
        createTestableBounds(space), createProjectableBounds(space), timelimit,
        collisionResolution);

    return untimedTrajectory;
  }

  return nullptr;
}

//==============================================================================
InterpolatedPtr planToTSR(const MetaSkeletonPtr &metaSkeleton,
                          const MetaSkeletonStateSpacePtr &space,
                          const BodyNodePtr &bn, const TSRPtr &tsr,
                          int maxNumTrials, double timelimit,
                          const TestablePtr &collisionTestable, RNG *rng,
                          double collisionResolution) {
  using constraint::InverseKinematicsSampleable;

  // Convert TSR constraint into IK constraint
  InverseKinematicsSampleable ikSampleable(
      space, metaSkeleton, tsr,
      createSampleableBounds(space, rng->clone()),
      InverseKinematics::create(bn), maxNumTrials);

  auto generator = ikSampleable.createSampleGenerator();

  // Current state
  auto startState = space->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  // Goal state
  auto goalState = space->createState();

  Eigen::VectorXd goal;

  // TODO: Change this to timelimit once we use a fail-fast planner
  double timelimitPerSample = timelimit / maxNumTrials;

  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // HACK: try lots of snap plans first
  static const std::size_t maxSnapSamples{100};
  std::size_t snapSamples = 0;
  while (snapSamples < maxSnapSamples && generator->canSample()) {
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
        space, startState, goalState,
        std::make_shared<GeodesicInterpolator>(space), collisionTestable,
        pResult);

    if (traj)
      return traj;
  }

  auto robot = metaSkeleton->getBodyNode(0)->getRobot();

  // Start the timer
  dart::common::Timer timer;
  timer.start();
  while (timer.getElapsedTime() < timelimit && generator->canSample()) {
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

    auto traj = planToConfiguration(
        robot, metaSkeleton, space, goal,
        std::min(timelimitPerSample, timelimit - timer.getElapsedTime()),
        collisionTestable, rng, collisionResolution);

    if (traj)
      return traj;
  }

  return nullptr;
}

} // namespace util
} // namespace robot
} // namespace aikido
