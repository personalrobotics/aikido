#include "aikido/robot/ConcreteRobot.hpp"

#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/planner/SnapConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"

namespace aikido {
namespace robot {

using constraint::ConstTestablePtr;
using constraint::TestablePtr;
using constraint::dart::CollisionFreePtr;
using constraint::dart::TSRPtr;
using planner::ConfigurationToConfiguration;
using planner::SnapConfigurationToConfigurationPlanner;
using planner::TrajectoryPostProcessor;
using planner::kunzretimer::KunzRetimer;
using planner::parabolic::ParabolicSmoother;
using planner::parabolic::ParabolicTimer;
using statespace::GeodesicInterpolator;
using statespace::StateSpace;
using statespace::StateSpacePtr;
using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSaver;
using statespace::dart::MetaSkeletonStateSpace;
using statespace::dart::MetaSkeletonStateSpacePtr;
using trajectory::Interpolated;
using trajectory::InterpolatedPtr;
using trajectory::Spline;
using trajectory::TrajectoryPtr;
using trajectory::UniqueSplinePtr;

using dart::dynamics::BodyNodePtr;
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
TrajectoryPtr ConcreteRobot::planToConfiguration(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr& metaSkeleton,
    const StateSpace::State* goalState,
    const CollisionFreePtr& collisionFree,
    double timelimit)
{
  DART_UNUSED(timelimit);

  auto robot = metaSkeleton->getBodyNode(0)->getSkeleton();
  std::lock_guard<std::mutex> lock(robot->getMutex());
  // Save the current state of the space
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  auto collisionConstraint
      = getFullCollisionConstraint(stateSpace, metaSkeleton, collisionFree);

  auto snapConfigToConfigPlanner
      = std::make_shared<SnapConfigurationToConfigurationPlanner>(
          stateSpace, std::make_shared<GeodesicInterpolator>(stateSpace));

  auto startState
      = stateSpace->getScopedStateFromMetaSkeleton(metaSkeleton.get());
  auto problem = ConfigurationToConfiguration(
      stateSpace, startState, goalState, collisionFree);

  return snapConfigToConfigPlanner->plan(problem);
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
  return util::planToConfigurations(
      stateSpace,
      metaSkeleton,
      goalStates,
      collisionFree,
      cloneRNG().get(),
      timelimit);
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

  return util::planToTSR(
      stateSpace,
      metaSkeleton,
      bn,
      tsr,
      collisionConstraint,
      cloneRNG().get(),
      timelimit,
      maxNumTrials,
      ranker);
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

  // Uses CRRT.
  return util::planToTSRwithTrajectoryConstraint(
      stateSpace,
      metaSkeleton,
      bodyNode,
      goalTsr,
      constraintTsr,
      collisionConstraint,
      timelimit,
      mCRRTParameters);
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
