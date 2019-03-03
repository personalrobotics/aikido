#include "aikido/robot/util.hpp"

#include <algorithm>
#include <dart/common/Console.hpp>
#include <dart/common/StlHelpers.hpp>
#include <dart/common/Timer.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/CyclicSampleable.hpp"
#include "aikido/constraint/FiniteSampleable.hpp"
#include "aikido/constraint/SequentialSampleable.hpp"

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
#include "aikido/planner/ompl/Planner.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"
#include "aikido/planner/vectorfield/VectorFieldPlanner.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

#include "aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp"

namespace aikido {
namespace robot {
namespace util {

using common::cloneRNGFrom;
using common::RNG;
using constraint::dart::CollisionFreePtr;
using constraint::dart::InverseKinematicsSampleable;
using constraint::dart::TSR;
using constraint::dart::TSRPtr;
using constraint::TestablePtr;
using constraint::SequentialSampleable;
using constraint::dart::createProjectableBounds;
using constraint::dart::createSampleableBounds;
using constraint::dart::createTestableBounds;
using distance::createDistanceMetric;
using distance::ConstConfigurationRankerPtr;
using distance::NominalConfigurationRanker;
using planner::ConfigurationToConfiguration;
using planner::SnapConfigurationToConfigurationPlanner;
using planner::ompl::OMPLConfigurationToConfigurationPlanner;
using statespace::GeodesicInterpolator;
using statespace::dart::MetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSaver;
using statespace::dart::MetaSkeletonStateSpace;
using statespace::StateSpace;
using trajectory::TrajectoryPtr;
using trajectory::Interpolated;
using trajectory::InterpolatedPtr;
using trajectory::SplinePtr;

using dart::collision::FCLCollisionDetector;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::ChainPtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;

static const double collisionResolution = 0.1;

//==============================================================================
trajectory::TrajectoryPtr planToConfiguration(
    const MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const StateSpace::State* goalState,
    const TestablePtr& collisionTestable,
    RNG* rng,
    double timelimit)
{
  DART_UNUSED(timelimit);

  using planner::ompl::planOMPL;
  using planner::ConfigurationToConfiguration;
  using planner::SnapConfigurationToConfigurationPlanner;

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // First test with Snap Planner
  SnapConfigurationToConfigurationPlanner::Result pResult;
  TrajectoryPtr untimedTrajectory;

  auto startState = space->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  auto problem = ConfigurationToConfiguration(
      space, startState, goalState, collisionTestable);
  auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      space, std::make_shared<GeodesicInterpolator>(space));
  untimedTrajectory = planner->plan(problem, &pResult);

  // Return if the trajectory is non-empty
  if (untimedTrajectory)
    return untimedTrajectory;

  auto plannerOMPL = std::
      make_shared<OMPLConfigurationToConfigurationPlanner<::ompl::geometric::
                                                              RRTConnect>>(
          space, rng);

  untimedTrajectory = plannerOMPL->plan(problem, &pResult);

  return untimedTrajectory;
}

//==============================================================================
trajectory::TrajectoryPtr planToConfigurations(
    const MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const std::vector<StateSpace::State*>& goalStates,
    const TestablePtr& collisionTestable,
    RNG* rng,
    double timelimit)
{
  using planner::ompl::planOMPL;
  DART_UNUSED(timelimit);

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  auto startState = space->getScopedStateFromMetaSkeleton(metaSkeleton.get());
  SnapConfigurationToConfigurationPlanner::Result pResult;
  auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      space, std::make_shared<GeodesicInterpolator>(space));

  for (const auto& goalState : goalStates)
  {
    // First test with Snap Planner
    auto problem = ConfigurationToConfiguration(
        space, startState, goalState, collisionTestable);
    TrajectoryPtr untimedTrajectory = planner->plan(problem, &pResult);

    // Return if the trajectory is non-empty
    if (untimedTrajectory)
      return untimedTrajectory;

    auto plannerOMPL = std::
        make_shared<OMPLConfigurationToConfigurationPlanner<::ompl::geometric::
                                                                RRTConnect>>(
            space, rng);

    untimedTrajectory = plannerOMPL->plan(problem, &pResult);

    if (untimedTrajectory)
      return untimedTrajectory;
  }

  return nullptr;
}

//==============================================================================
trajectory::TrajectoryPtr planToTSR(
    const MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const BodyNodePtr& bn,
    const TSRPtr& tsr,
    const TestablePtr& collisionTestable,
    RNG* rng,
    double timelimit,
    std::size_t maxNumTrials,
    const distance::ConstConfigurationRankerPtr& ranker)
{
  dart::common::Timer timer;
  timer.start();

  // Create an IK solver with metaSkeleton dofs.
  auto ik = InverseKinematics::create(bn);

  // TODO: DART may be updated to check for single skeleton
  if (metaSkeleton->getNumDofs() == 0)
    throw std::invalid_argument("MetaSkeleton has 0 degrees of freedom.");

  auto skeleton = metaSkeleton->getDof(0)->getSkeleton();
  for (size_t i = 1; i < metaSkeleton->getNumDofs(); ++i)
  {
    if (metaSkeleton->getDof(i)->getSkeleton() != skeleton)
      throw std::invalid_argument("MetaSkeleton has more than 1 skeleton.");
  }

  ik->setDofs(metaSkeleton->getDofs());

  auto startState = space->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  std::cout << "Start positions " << metaSkeleton->getPositions().transpose() << std::endl;

  // Hardcoded seed for ada
  std::vector<MetaSkeletonStateSpace::ScopedState> seedStates;
  Eigen::VectorXd seedConfiguration(space->getDimension());
  seedStates.emplace_back(startState.clone());

  space->convertStateToPositions(startState, seedConfiguration);

  auto seedState = space->createState();
  seedConfiguration << 0.71149, 1.96881, 2.12461, -1.60078, -2.06181, -2.33079;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -5.34054, 1.9574, 1.86268, 0.0843705, -4.48063, 5.66238;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.3927, 4.3067, 3.94881, -4.62768, -2.25937, -2.07654;

  seedState = space->createState();
  seedConfiguration << -2.39079, 4.30507, 3.96979, 1.65921, -2.25655, -2.08021;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.46508, 4.35337, 3.95698, 1.76356, -2.28537, -1.9966;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.18807, 3.26458, 1.77104, -2.4695, -2.02196, -2.07969;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.14454, 3.23998, 1.60007, -2.57557, -1.81261, 2.99757;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.1928, 3.40455, 1.84249, -2.52313, -1.92402, 3.76996;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.32828, 4.19546, 3.72812, 1.71219, -2.42468, 4.38807;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.1928, 3.40455, 1.84249, -2.52313, -1.92402, -2.51322;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -1.4778, 2.92522, 1.00283, -2.08638,  1.44895,  1.32235;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.06047, 3.33506, 1.90762, -2.4133, -2.11661, -0.682215;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration <<-2.35264, 4.24509, 3.96006, 1.58616, -2.21484, -2.15233;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -1.47776, 2.92554, 1.00349, -2.08592, 1.44893, 1.3223;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -3.14042, 3.15514, 1.16615, 0.573695, 1.59742, -1.28865;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.40444, 3.31599, 1.88258, -0.724555, 2.11118, -1.53751;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << 0.753014, 1.9752, 1.89462, 0.238096, 1.83547, -0.636708;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.32925, 4.22822, 3.84025, 1.66769, -2.3464, -1.99578;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -1.99374, 3.30963, 1.8211, -2.46515, -2.03034, -2.25597;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.15136, 3.41949, 1.69291, -2.6289, -1.69653, -2.03035;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedState = space->createState();
  seedConfiguration << -2.36154, 4.24923, 3.86001, 1.69034, -2.33204, -1.9959;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.32705, 4.18856, 3.72915, 1.7198, -2.41678, 10.6595;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration <<-2.44866, 4.33757, 3.93095, 1.74906, -2.2925, -1.99095;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -0.90715, 4.10547, 4.81739, 1.75586, -2.27429, -2.01147 ;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.64253, 2.88948, 1.10437, 0.0433776, 1.1455, 2.04545;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.77921, 2.84293, 1.07646,  -0.0206763, 1.09828, 2.25931;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -5.39202, 1.94476, 1.82651, 0.14991, 1.78208, -0.637554;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.34241, 4.20543, 3.84994, 1.64344, -2.3003, -2.04766 ;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -1.99782, 3.35221, 1.84423, -2.48742, -1.99844, -1.91277;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -1.63667, 2.94542, 0.962171, 2.92917, -1.22056,  1.81286;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -1.81719,  2.82256,  1.56441,  2.92916, -1.22059, 1.81307;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << 0.546467, 1.96717, 1.72055, 0.503852, 1.73615, -0.751649;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -1.0798, 2.72718, 1.74161, -1.58974,  1.51289, 0.913531;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -1.9401, 3.92174, 3.38932, -3.75006, 2.59724, -0.817528;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -1.48081, 4.19045, 4.09752, 1.70698, -2.03137, -2.32713;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.13102, 3.54862, 1.99171, -2.52359, -1.92639, -2.06387;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -1.66161, 3.18774, 1.69401, 3.23128, -1.80354, -4.45043;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.11237, 3.29138, 1.93036, -2.3675, -2.23003, -0.79868;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.14079, 3.30573, 1.94499, -2.3777, -2.18687, -5.28631;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.12146, 3.94101, 3.34574, 2.99502, -3.92992, -0.440412;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -8.07629, 3.47886, 2.15987, -2.64209, -2.53331, 2.74152;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.12611, 3.30686, 2.03593, -2.27111, -2.34318, 3.92119;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.19111, 3.32611, 2.0867, -2.19583, -2.42988, 2.93824;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.1018, 3.44181, 1.90086, -2.50312, -1.95277, 6.93331;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << 0.816582, 2.88355, 3.96263, 1.86817, 2.83235, 4.47004;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -1.81795, 3.19223, 1.24379, 2.93106, -1.18668, 7.97186;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << 0.871881, 2.89292, 4.01451, 7.52685, -2.7644, 8.56144;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << 0.761478, 2.9263, 4.02089, 1.89851, 2.81683, 0.202515;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << 0.531097, 2.06645, 1.60537, 0.602812, 1.60736, -0.878579;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -5.5214, 2.92906, 4.00137, 1.90192,  2.81911, 0.205562;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.32382,  3.3865,  2.30347, 4.8859, 2.9721, -7.43382;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << 6.90501, 2.89118, 4.4486, 2.51436, 1.92913, -7.63507;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.30184, 4.24077, 3.87564, -4.67058, 3.94133, -8.32409;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -14.6813,  4.24396,  4.22699, 3.08014, 1.93032, 5.6674;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -14.8694,  3.37569,  2.29006, -1.41612,  2.96088,  11.4257 ;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  seedConfiguration << -2.13142, 3.31094, 2.04218, -2.27228, 3.94973,  3.93241;
  space->convertPositionsToState(seedConfiguration, seedState);
  seedStates.emplace_back(seedState.clone());

  auto finiteSeedSampleable
      = std::make_shared<constraint::FiniteSampleable>(space, seedStates);

  std::vector<constraint::ConstSampleablePtr> sampleableVector;
  sampleableVector.push_back(finiteSeedSampleable);
  sampleableVector.push_back(createSampleableBounds(space, rng->clone()));
  auto seedSampleable = std::make_shared<constraint::SequentialSampleable>(
      space, sampleableVector);

  // Convert TSR constraint into IK constraint
  InverseKinematicsSampleable ikSampleable(
      space,
      metaSkeleton,
      tsr,
      seedSampleable,
      ik,
      maxNumTrials);

  auto generator = ikSampleable.createSampleGenerator();

  // Goal state
  auto goalState = space->createState();

  // TODO: Change this to timelimit once we use a fail-fast planner
  double timelimitPerSample = timelimit / maxNumTrials;

  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // HACK: try lots of snap plans first
  static const std::size_t maxSnapSamples{10};

  for (std::size_t batchIdx = 0; batchIdx <= 10; ++batchIdx)
  {
    std::cout << "Batch " << batchIdx << "/10" << std::endl;
    std::size_t snapSamples = 0;

    auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
    SnapConfigurationToConfigurationPlanner::Result pResult;
    auto snapPlanner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
        space, std::make_shared<GeodesicInterpolator>(space));

    std::vector<MetaSkeletonStateSpace::ScopedState> configurations;

    // Use a ranker
    ConstConfigurationRankerPtr configurationRanker(ranker);
    if (!ranker)
    {
      auto nominalState = space->createState();
      space->copyState(startState, nominalState);
      configurationRanker = std::make_shared<const NominalConfigurationRanker>(
          space, metaSkeleton, std::vector<double>(), std::move(nominalState));
    }

    while (snapSamples < maxSnapSamples && generator->canSample())
    {
      // Sample from TSR
      std::lock_guard<std::mutex> lock(robot->getMutex());
      bool sampled = generator->sample(goalState);

      // Increment even if it's not a valid sample since this loop
      // has to terminate even if none are valid.
      ++snapSamples;

      if (!sampled)
        continue;

      configurations.emplace_back(goalState.clone());
    }

    if (configurations.empty())
    {
      std::cout <<"Failed to get any valid IK sample on batch " << batchIdx << std::endl;
      continue;
    }

    configurationRanker->rankConfigurations(configurations);

    Eigen::VectorXd _positions;
    space->convertStateToPositions(startState, _positions);
    std::cout << "Snap initial " << _positions.transpose() << std::endl;

    // Try snap planner first
    for (std::size_t i = 0; i < configurations.size(); ++i)
    {
      auto problem = ConfigurationToConfiguration(
          space, startState, configurations[i], collisionTestable);

      auto traj = snapPlanner->plan(problem, &pResult);

      if (traj)
      {
        std::cout << "Snap succeeded with " << i << "th sample " << std::endl;
        std:: cout << "Took " << timer.getElapsedTime() << " seconds." << std::endl;
        return traj;
      }
    }

  }

  Eigen::VectorXd positions;
  space->convertStateToPositions(startState, positions);
  std::cout << "Snap Failed " << positions.transpose() << std::endl;
  std:: cout << "Took " << timer.getElapsedTime() << " seconds." << std::endl;
  return nullptr;

  /*
  // Start the timer
  dart::common::Timer timer;
  timer.start();
  for (std::size_t i = 0; i < configurations.size(); ++i)
  {
    auto problem = ConfigurationToConfiguration(
        space, startState, configurations[i], collisionTestable);

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
  */
}

//==============================================================================
InterpolatedPtr planToTSRwithTrajectoryConstraint(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const TSRPtr& goalTsr,
    const TSRPtr& constraintTsr,
    const TestablePtr& collisionTestable,
    double timelimit,
    const CRRTPlannerParameters& crrtParameters)
{
  using aikido::constraint::Sampleable;
  using aikido::constraint::dart::InverseKinematicsSampleable;
  using aikido::constraint::CyclicSampleable;
  using aikido::constraint::dart::FrameDifferentiable;
  using aikido::constraint::dart::FrameTestable;
  using aikido::constraint::NewtonsMethodProjectable;
  using aikido::planner::ompl::planCRRTConnect;

  std::size_t projectionMaxIteration = crrtParameters.projectionMaxIteration;
  double projectionTolerance = crrtParameters.projectionTolerance;

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());

  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // Create seed constraint
  std::shared_ptr<Sampleable> seedConstraint
      = createSampleableBounds(space, crrtParameters.rng->clone());

  // TODO: DART may be updated to check for single skeleton
  if (metaSkeleton->getNumDofs() == 0)
    throw std::invalid_argument("MetaSkeleton has 0 degrees of freedom.");

  auto skeleton = metaSkeleton->getDof(0)->getSkeleton();
  for (size_t i = 1; i < metaSkeleton->getNumDofs(); ++i)
  {
    if (metaSkeleton->getDof(i)->getSkeleton() != skeleton)
      throw std::invalid_argument("MetaSkeleton has more than 1 skeleton.");
  }

  // Create an IK solver with metaSkeleton dofs
  auto ik = InverseKinematics::create(bodyNode);

  ik->setDofs(metaSkeleton->getDofs());

  // create goal sampleable
  auto goalSampleable = std::make_shared<InverseKinematicsSampleable>(
      space,
      metaSkeleton,
      std::make_shared<CyclicSampleable>(goalTsr),
      seedConstraint,
      ik,
      crrtParameters.maxNumTrials);

  // create goal testable
  auto goalTestable = std::make_shared<FrameTestable>(
      space, metaSkeleton, bodyNode.get(), goalTsr);

  // create constraint sampleable
  auto constraintSampleable = std::make_shared<InverseKinematicsSampleable>(
      space,
      metaSkeleton,
      constraintTsr,
      seedConstraint,
      ik,
      crrtParameters.maxNumTrials);

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
      crrtParameters.maxExtensionDistance,
      crrtParameters.maxDistanceBtwProjections,
      crrtParameters.minStepSize,
      crrtParameters.minTreeConnectionDistance);

  return traj;
}

//==============================================================================
trajectory::TrajectoryPtr planToEndEffectorOffset(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const Eigen::Vector3d& direction,
    const TestablePtr& collisionTestable,
    double distance,
    double timelimit,
    double positionTolerance,
    double angularTolerance,
    const VectorFieldPlannerParameters& vfParameters,
    const CRRTPlannerParameters& crrtParameters)
{
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  auto startState = space->createState();
  space->getState(metaSkeleton.get(), startState);

  auto minDistance
      = std::max(0.0, distance - vfParameters.negativeDistanceTolerance);
  auto maxDistance = distance + vfParameters.positiveDistanceTolerance;

  auto traj = planner::vectorfield::planToEndEffectorOffset(
      space,
      *startState,
      metaSkeleton,
      bodyNode,
      collisionTestable,
      direction,
      minDistance,
      maxDistance,
      positionTolerance,
      angularTolerance,
      vfParameters.initialStepSize,
      vfParameters.jointLimitTolerance,
      vfParameters.constraintCheckResolution,
      std::chrono::duration<double>(timelimit));

  return std::move(traj);

  /*
  return planToEndEffectorOffsetByCRRT(
      space,
      metaSkeleton,
      bodyNode,
      collisionTestable,
      direction,
      distance,
      timelimit,
      positionTolerance,
      angularTolerance,
      crrtParameters);
      */
}

//==============================================================================
InterpolatedPtr planToEndEffectorOffsetByCRRT(
    const MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const BodyNodePtr& bodyNode,
    const TestablePtr& collisionTestable,
    const Eigen::Vector3d& direction,
    double distance,
    double timelimit,
    double positionTolerance,
    double angularTolerance,
    const CRRTPlannerParameters& crrtParameters)
{
  // if direction vector is a zero vector
  if (direction.norm() == 0.0)
  {
    throw std::runtime_error("Direction vector is a zero vector");
  }

  // normalize direction vector
  Eigen::Vector3d directionCopy = direction;
  directionCopy.normalize();

  if (distance < 0.0)
  {
    distance = -distance;
    directionCopy = -directionCopy;
  }

  auto goalTsr = std::make_shared<TSR>();
  auto constraintTsr = std::make_shared<TSR>();
  bool success = getGoalAndConstraintTSRForEndEffectorOffset(
      bodyNode,
      directionCopy,
      distance,
      goalTsr,
      constraintTsr,
      positionTolerance,
      angularTolerance);

  if (!success)
    throw std::runtime_error("failed in creating TSR");

  auto untimedTrajectory = planToTSRwithTrajectoryConstraint(
      space,
      metaSkeleton,
      bodyNode,
      goalTsr,
      constraintTsr,
      collisionTestable,
      timelimit,
      crrtParameters);

  return untimedTrajectory;
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
