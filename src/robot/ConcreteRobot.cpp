#include "aikido/robot/ConcreteRobot.hpp"

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "aikido/constraint/CyclicSampleable.hpp"
#include "aikido/constraint/FiniteSampleable.hpp"
#include "aikido/constraint/NewtonsMethodProjectable.hpp"
#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/distance/NominalConfigurationRanker.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include "aikido/planner/SnapConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"


namespace aikido {
namespace robot {

using constraint::ConstTestablePtr;
using  constraint::CyclicSampleable;

using constraint::TestablePtr;
using constraint::dart::CollisionFreePtr;
using constraint::dart::createProjectableBounds;
using constraint::dart::createSampleableBounds;
using constraint::dart::createTestableBounds;
using constraint::dart::TSRPtr;
using constraint::dart::InverseKinematicsSampleable;
using constraint::dart::FrameDifferentiable;
using constraint::dart::FrameTestable;
using constraint::NewtonsMethodProjectable;
using constraint::Sampleable;
using distance::ConstConfigurationRankerPtr;
using distance::createDistanceMetric;
using distance::NominalConfigurationRanker;
using planner::TrajectoryPostProcessor;
using planner::kunzretimer::KunzRetimer;
using planner::parabolic::ParabolicSmoother;
using planner::parabolic::ParabolicTimer;
using planner::ConfigurationToConfiguration;
using planner::SnapConfigurationToConfigurationPlanner;
using planner::ompl::planCRRTConnect;
using planner::ompl::planOMPL;
using planner::ompl::OMPLConfigurationToConfigurationPlanner;
using statespace::GeodesicInterpolator;
using statespace::StateSpace;
using statespace::StateSpacePtr;
using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSpace;
using statespace::dart::MetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSaver;
using trajectory::Interpolated;
using trajectory::InterpolatedPtr;
using trajectory::Spline;
using trajectory::TrajectoryPtr;
using trajectory::UniqueSplinePtr;

using dart::dynamics::BodyNodePtr;
using dart::dynamics::InverseKinematics;
using dart::dynamics::MetaSkeleton;
using dart::dynamics::MetaSkeletonPtr;






// TODO: Temporary constants for planning calls.
// These should be defined when we construct planner adapter classes
// static const double collisionResolution = 0.1;
static const double asymmetryTolerance = 1e-3;

namespace {

// TODO: These may not generalize to many robots.
Eigen::VectorXd getSymmetricLimits(
    const MetaSkeleton& metaSkeleton,
    const Eigen::VectorXd& lowerLimits,
    const Eigen::VectorXd& upperLimits,
    const std::string& limitName,
    double asymmetryTolerance)
{
  const auto numDofs = metaSkeleton.getNumDofs();
  assert(static_cast<std::size_t>(lowerLimits.size()) == numDofs);
  assert(static_cast<std::size_t>(upperLimits.size()) == numDofs);

  Eigen::VectorXd symmetricLimits(numDofs);
  for (std::size_t iDof = 0; iDof < numDofs; ++iDof)
  {
    symmetricLimits[iDof] = std::min(-lowerLimits[iDof], upperLimits[iDof]);
    if (std::abs(lowerLimits[iDof] + upperLimits[iDof]) > asymmetryTolerance)
    {
      dtwarn << "MetaSkeleton '" << metaSkeleton.getName()
             << "' has asymmetric " << limitName << " limits ["
             << lowerLimits[iDof] << ", " << upperLimits[iDof]
             << "] for DegreeOfFreedom '"
             << metaSkeleton.getDof(iDof)->getName() << "' (index: " << iDof
             << "). Using a conservative limit of" << symmetricLimits[iDof]
             << ".";
    }
  }
  return symmetricLimits;
}

Eigen::VectorXd getSymmetricVelocityLimits(
    const MetaSkeleton& metaSkeleton, double asymmetryTolerance)
{
  return getSymmetricLimits(
      metaSkeleton,
      metaSkeleton.getVelocityLowerLimits(),
      metaSkeleton.getVelocityUpperLimits(),
      "velocity",
      asymmetryTolerance);
}

Eigen::VectorXd getSymmetricAccelerationLimits(
    const MetaSkeleton& metaSkeleton, double asymmetryTolerance)
{
  return getSymmetricLimits(
      metaSkeleton,
      metaSkeleton.getAccelerationLowerLimits(),
      metaSkeleton.getAccelerationUpperLimits(),
      "acceleration",
      asymmetryTolerance);
}

} // namespace

//==============================================================================
ConcreteRobot::ConcreteRobot(
    const std::string& name,
    MetaSkeletonPtr metaSkeleton,
    bool /*simulation*/,
    common::UniqueRNGPtr rng,
    control::TrajectoryExecutorPtr trajectoryExecutor,
    dart::collision::CollisionDetectorPtr collisionDetector,
    std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
        selfCollisionFilter)
  : mRootRobot(this)
  , mName(name)
  , mMetaSkeleton(metaSkeleton)
  , mStateSpace(std::make_shared<MetaSkeletonStateSpace>(mMetaSkeleton.get()))
  , mParentSkeleton(nullptr)
  // , mSimulation(simulation)
  , mRng(std::move(rng))
  , mTrajectoryExecutor(std::move(trajectoryExecutor))
  // , mCollisionResolution(collisionResolution)
  , mCollisionDetector(collisionDetector)
  , mSelfCollisionFilter(selfCollisionFilter)
{
  if (!mMetaSkeleton)
    throw std::invalid_argument("Robot is nullptr.");

  mParentSkeleton = mMetaSkeleton->getBodyNode(0)->getSkeleton();
}

//==============================================================================
UniqueSplinePtr ConcreteRobot::smoothPath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path,
    const constraint::TestablePtr& constraint)
{
  Eigen::VectorXd velocityLimits = getVelocityLimits(*metaSkeleton);
  Eigen::VectorXd accelerationLimits = getAccelerationLimits(*metaSkeleton);
  auto smoother
      = std::make_shared<ParabolicSmoother>(velocityLimits, accelerationLimits);

  auto interpolated = dynamic_cast<const Interpolated*>(path);
  if (interpolated)
    return smoother->postprocess(
        *interpolated, *(cloneRNG().get()), constraint);

  auto spline = dynamic_cast<const Spline*>(path);
  if (spline)
    return smoother->postprocess(*spline, *(cloneRNG().get()), constraint);

  throw std::invalid_argument("Path should be either Spline or Interpolated.");
}

//==============================================================================
UniqueSplinePtr ConcreteRobot::retimePath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path)
{
  Eigen::VectorXd velocityLimits = getVelocityLimits(*metaSkeleton);
  Eigen::VectorXd accelerationLimits = getAccelerationLimits(*metaSkeleton);
  auto retimer
      = std::make_shared<ParabolicTimer>(velocityLimits, accelerationLimits);

  auto interpolated = dynamic_cast<const Interpolated*>(path);
  if (interpolated)
    return retimer->postprocess(*interpolated, *(cloneRNG().get()));

  auto spline = dynamic_cast<const Spline*>(path);
  if (spline)
    return retimer->postprocess(*spline, *(cloneRNG().get()));

  throw std::invalid_argument("Path should be either Spline or Interpolated.");
}

//==============================================================================
UniqueSplinePtr ConcreteRobot::retimePathWithKunz(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path,
    double maxDeviation,
    double timestep)
{
  Eigen::VectorXd velocityLimits = getVelocityLimits(*metaSkeleton);
  Eigen::VectorXd accelerationLimits = getAccelerationLimits(*metaSkeleton);
  auto retimer = std::make_shared<KunzRetimer>(
      velocityLimits, accelerationLimits, maxDeviation, timestep);

  auto interpolated = dynamic_cast<const Interpolated*>(path);
  if (interpolated)
    return retimer->postprocess(*interpolated, *(cloneRNG().get()));

  auto spline = dynamic_cast<const Spline*>(path);
  if (spline)
    return retimer->postprocess(*spline, *(cloneRNG().get()));

  throw std::invalid_argument("Path should be either Spline or Interpolated.");
}

//==============================================================================
std::future<void> ConcreteRobot::executeTrajectory(
    const TrajectoryPtr& trajectory) const
{
  return mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
boost::optional<Eigen::VectorXd> ConcreteRobot::getNamedConfiguration(
    const std::string& name) const
{
  auto configuration = mNamedConfigurations.find(name);
  if (configuration == mNamedConfigurations.end())
    return boost::none;

  return configuration->second;
}

//==============================================================================
void ConcreteRobot::setNamedConfigurations(
    std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations)
{
  mNamedConfigurations = std::move(namedConfigurations);
}

//==============================================================================
std::string ConcreteRobot::getName() const
{
  return mName;
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr ConcreteRobot::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

//==============================================================================
statespace::dart::ConstMetaSkeletonStateSpacePtr ConcreteRobot::getStateSpace()
    const
{
  return mStateSpace;
}

//=============================================================================
void ConcreteRobot::setRoot(Robot* robot)
{
  if (robot == nullptr)
    throw std::invalid_argument("ConcreteRobot is null.");

  mRootRobot = robot;
}

//==============================================================================
void ConcreteRobot::step(const std::chrono::system_clock::time_point& timepoint)
{
  // Assumes that the parent robot is locked
  mTrajectoryExecutor->step(timepoint);
}

// ==============================================================================
CollisionFreePtr ConcreteRobot::getSelfCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton) const
{
  using constraint::dart::CollisionFree;

  if (mRootRobot != this)
    return mRootRobot->getSelfCollisionConstraint(space, metaSkeleton);

  mParentSkeleton->enableSelfCollisionCheck();
  mParentSkeleton->disableAdjacentBodyCheck();

  // TODO: Switch to PRIMITIVE once this is fixed in DART.
  // mCollisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  auto collisionOption
      = dart::collision::CollisionOption(false, 1, mSelfCollisionFilter);
  auto collisionFreeConstraint = std::make_shared<CollisionFree>(
      space, metaSkeleton, mCollisionDetector, collisionOption);
  collisionFreeConstraint->addSelfCheck(
      mCollisionDetector->createCollisionGroupAsSharedPtr(mMetaSkeleton.get()));
  return collisionFreeConstraint;
}

//=============================================================================
TestablePtr ConcreteRobot::getFullCollisionConstraint(
    const ConstMetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const CollisionFreePtr& collisionFree) const
{
  using constraint::TestableIntersection;

  if (mRootRobot != this)
    return mRootRobot->getFullCollisionConstraint(
        space, metaSkeleton, collisionFree);

  auto selfCollisionFree = getSelfCollisionConstraint(space, metaSkeleton);

  if (!collisionFree)
    return selfCollisionFree;

  // Make testable constraints for collision check
  std::vector<ConstTestablePtr> constraints;
  constraints.reserve(2);
  constraints.emplace_back(selfCollisionFree);
  if (collisionFree)
  {
    if (collisionFree->getStateSpace() != space)
    {
      throw std::runtime_error("CollisionFree has incorrect statespace.");
    }
    constraints.emplace_back(collisionFree);
  }

  return std::make_shared<TestableIntersection>(space, constraints);
}

//==============================================================================
std::shared_ptr<TrajectoryPostProcessor>
ConcreteRobot::getTrajectoryPostProcessor(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    bool enableShortcut,
    bool enableBlend,
    double shortcutTimelimit,
    double blendRadius,
    int blendIterations,
    double feasibilityCheckResolution,
    double feasibilityApproxTolerance) const
{
  Eigen::VectorXd velocityLimits = getVelocityLimits(*metaSkeleton);
  Eigen::VectorXd accelerationLimits = getAccelerationLimits(*metaSkeleton);

  return std::make_shared<ParabolicSmoother>(
      velocityLimits,
      accelerationLimits,
      enableShortcut,
      enableBlend,
      shortcutTimelimit,
      blendRadius,
      blendIterations,
      feasibilityCheckResolution,
      feasibilityApproxTolerance);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToConfiguration(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const StateSpace::State* goalState,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  DART_UNUSED(timelimit);


  auto collisionConstraint
      = getFullCollisionConstraint(stateSpace, metaSkeleton, collisionFree);

  std::lock_guard<std::mutex> lock(metaSkeleton->getBodyNode(0)->getSkeleton()->getMutex());
  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // First test with Snap Planner
  SnapConfigurationToConfigurationPlanner::Result pResult;
  TrajectoryPtr untimedTrajectory;

  auto startState = stateSpace->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  auto problem = ConfigurationToConfiguration(
      stateSpace, startState, goalState, collisionConstraint);
  auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      stateSpace, std::make_shared<GeodesicInterpolator>(stateSpace));
  untimedTrajectory = planner->plan(problem, &pResult);

  // Return if the trajectory is non-empty
  if (untimedTrajectory)
    return untimedTrajectory;

  auto plannerOMPL = std::make_shared<
      OMPLConfigurationToConfigurationPlanner<::ompl::geometric::RRTConnect>>(
      stateSpace, cloneRNG().get());

  untimedTrajectory = plannerOMPL->plan(problem, &pResult);

  return untimedTrajectory;
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToConfiguration(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const Eigen::VectorXd& goal,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  auto goalState = stateSpace->createState();
  stateSpace->convertPositionsToState(goal, goalState);

  return planToConfiguration(
      stateSpace, metaSkeleton, goalState, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToConfigurations(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const std::vector<StateSpace::State*>& goalStates,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  using planner::ompl::planOMPL;
  DART_UNUSED(timelimit);

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  auto startState = stateSpace->getScopedStateFromMetaSkeleton(metaSkeleton.get());
  SnapConfigurationToConfigurationPlanner::Result pResult;
  auto planner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      stateSpace, std::make_shared<GeodesicInterpolator>(stateSpace));

  for (const auto& goalState : goalStates)
  {
    // First test with Snap Planner
    auto problem = ConfigurationToConfiguration(
        stateSpace, startState, goalState, collisionFree);
    TrajectoryPtr untimedTrajectory = planner->plan(problem, &pResult);

    // Return if the trajectory is non-empty
    if (untimedTrajectory)
      return untimedTrajectory;

    auto plannerOMPL = std::make_shared<
        OMPLConfigurationToConfigurationPlanner<::ompl::geometric::RRTConnect>>(
        stateSpace, cloneRNG().get());

    untimedTrajectory = plannerOMPL->plan(problem, &pResult);

    if (untimedTrajectory)
      return untimedTrajectory;
  }

  return nullptr;
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToConfigurations(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const std::vector<Eigen::VectorXd>& goals,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  std::vector<StateSpace::State*> goalStates;
  goalStates.reserve(goals.size());

  for (const auto& goal : goals)
  {
    auto goalState = stateSpace->createState();
    stateSpace->convertPositionsToState(goal, goalState);
    goalStates.emplace_back(goalState);
  }

  return planToConfigurations(
      stateSpace, metaSkeleton, goalStates, collisionFree, timelimit);
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToTSR(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const BodyNodePtr& bn,
    const TSRPtr& tsr,
    const CollisionFreePtr& collisionFree,
    double timelimit,
    std::size_t maxNumTrials,
    const distance::ConstConfigurationRankerPtr& ranker)
{
  auto collisionConstraint
      = getFullCollisionConstraint(stateSpace, metaSkeleton, collisionFree);
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

  // Convert TSR constraint into IK constraint
  InverseKinematicsSampleable ikSampleable(
      stateSpace,
      metaSkeleton,
      tsr,
      createSampleableBounds(stateSpace, cloneRNG()),
      ik,
      static_cast<int>(maxNumTrials));

  auto generator = ikSampleable.createSampleGenerator();

  // Goal state
  auto goalState = stateSpace->createState();

  auto startState = stateSpace->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  // TODO: Change this to timelimit once we use a fail-fast planner
  double timelimitPerSample = timelimit / maxNumTrials;

  // Save the current state of the stateSpace
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // HACK: try lots of snap plans first
  static const std::size_t maxSnapSamples{100};
  std::size_t snapSamples = 0;

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  SnapConfigurationToConfigurationPlanner::Result pResult;
  auto snapPlanner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
      stateSpace, std::make_shared<GeodesicInterpolator>(stateSpace));

  std::vector<MetaSkeletonStateSpace::ScopedState> configurations;

  // Use a ranker
  ConstConfigurationRankerPtr configurationRanker(ranker);
  if (!ranker)
  {
    auto nominalState = stateSpace->createState();
    stateSpace->copyState(startState, nominalState);
    configurationRanker = std::make_shared<const NominalConfigurationRanker>(
        stateSpace, metaSkeleton, nominalState);
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
    return nullptr;

  configurationRanker->rankConfigurations(configurations);

  // Try snap planner first
  for (std::size_t i = 0; i < configurations.size(); ++i)
  {
    auto problem = ConfigurationToConfiguration(
        stateSpace, startState, configurations[i], collisionConstraint);

    auto traj = snapPlanner->plan(problem, &pResult);

    if (traj)
      return traj;
  }

  // Start the timer
  dart::common::Timer timer;
  timer.start();
  for (std::size_t i = 0; i < configurations.size(); ++i)
  {
    auto problem = ConfigurationToConfiguration(
        stateSpace, startState, configurations[i], collisionConstraint);

    auto traj = planToConfiguration(
        stateSpace,
        metaSkeleton,
        goalState,
        collisionFree,
        std::min(timelimitPerSample, timelimit - timer.getElapsedTime()));

    if (traj)
      return traj;
  }
  return nullptr;
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToTSRwithTrajectoryConstraint(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const BodyNodePtr& bodyNode,
    const TSRPtr& goalTsr,
    const TSRPtr& constraintTsr,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  auto collisionConstraint
      = getFullCollisionConstraint(stateSpace, metaSkeleton, collisionFree);

  std::size_t projectionMaxIteration = mCRRTParameters.projectionMaxIteration;
  double projectionTolerance = mCRRTParameters.projectionTolerance;

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());

  // Save the current state of the stateSpace
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  // Create seed constraint
  std::shared_ptr<Sampleable> seedConstraint
      = createSampleableBounds(stateSpace, mCRRTParameters.rng->clone());

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
      stateSpace,
      metaSkeleton,
      std::make_shared<CyclicSampleable>(goalTsr),
      seedConstraint,
      ik,
      mCRRTParameters.maxNumTrials);

  // create goal testable
  auto goalTestable = std::make_shared<FrameTestable>(
      stateSpace, metaSkeleton, bodyNode.get(), goalTsr);

  // create constraint sampleable
  auto constraintSampleable = std::make_shared<InverseKinematicsSampleable>(
      stateSpace,
      metaSkeleton,
      constraintTsr,
      seedConstraint,
      ik,
      mCRRTParameters.maxNumTrials);

  // create constraint projectable
  auto frameDiff = std::make_shared<FrameDifferentiable>(
      stateSpace, metaSkeleton, bodyNode.get(), constraintTsr);

  std::vector<double> projectionToleranceVec(
      frameDiff->getConstraintDimension(), projectionTolerance);
  auto constraintProjectable = std::make_shared<NewtonsMethodProjectable>(
      frameDiff, projectionToleranceVec, projectionMaxIteration);

  // Current state
  auto startState = stateSpace->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  // Call planner
  auto traj = planCRRTConnect(
      startState,
      goalTestable,
      goalSampleable,
      constraintProjectable,
      stateSpace,
      std::make_shared<GeodesicInterpolator>(stateSpace),
      createDistanceMetric(stateSpace),
      constraintSampleable,
      collisionConstraint,
      createTestableBounds(stateSpace),
      createProjectableBounds(stateSpace),
      timelimit,
      mCRRTParameters.maxExtensionDistance,
      mCRRTParameters.maxDistanceBtwProjections,
      mCRRTParameters.minStepSize,
      mCRRTParameters.minTreeConnectionDistance);

  return traj;
}

//==============================================================================
TrajectoryPtr ConcreteRobot::planToNamedConfiguration(
    const std::string& name,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  if (mNamedConfigurations.find(name) == mNamedConfigurations.end())
    throw std::runtime_error(name + " does not exist.");

  auto configuration = mNamedConfigurations[name];
  auto goalState = mStateSpace->createState();
  mStateSpace->convertPositionsToState(configuration, goalState);

  return planToConfiguration(
      mStateSpace, mMetaSkeleton, goalState, collisionFree, timelimit);
}

//=============================================================================
void ConcreteRobot::setCRRTPlannerParameters(
    const util::CRRTPlannerParameters& crrtParameters)
{
  mCRRTParameters = crrtParameters;
}

//==============================================================================
Eigen::VectorXd ConcreteRobot::getVelocityLimits(
    const MetaSkeleton& metaSkeleton) const
{
  return getSymmetricVelocityLimits(metaSkeleton, asymmetryTolerance);
}

//==============================================================================
Eigen::VectorXd ConcreteRobot::getAccelerationLimits(
    const MetaSkeleton& metaSkeleton) const
{
  return getSymmetricAccelerationLimits(metaSkeleton, asymmetryTolerance);
}

//==============================================================================
std::unique_ptr<common::RNG> ConcreteRobot::cloneRNG()
{
  return mRng->clone();
}

} // namespace robot
} // namespace aikido
