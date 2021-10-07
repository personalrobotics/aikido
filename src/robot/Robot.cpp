#include "aikido/robot/Robot.hpp"

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/common/Console.hpp>

#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/planner/SnapConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToConfiguration.hpp"
#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToConfiguration.hpp"
#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp"
#include "aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp"
#include "aikido/planner/dart/ConfigurationToTSR.hpp"
#include "aikido/planner/vectorfield/VectorFieldConfigurationToEndEffectorOffsetPlanner.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace robot {

namespace internal {

inline aikido::control::TrajectoryExecutorPtr trajExecFromSkeleton(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton)
  {
    throw std::invalid_argument("Null MetaskeletonPtr");
  }
  return std::make_shared<
      aikido::control::KinematicSimulationTrajectoryExecutor>(skeleton);
}

} // namespace internal

//==============================================================================
Robot::Robot(
    dart::dynamics::SkeletonPtr skeleton,
    const std::string name,
    const aikido::control::TrajectoryExecutorPtr trajExecutor)
  : mName(name)
  , mMetaSkeleton(skeleton)
  , mDofs(util::dofNamesFromSkeleton(skeleton))
  , mCollisionDetector(dart::collision::FCLCollisionDetector::create())
  , mSelfCollisionFilter(
        std::make_shared<dart::collision::BodyNodeCollisionFilter>())
{
  auto controlledMetaSkeleton
      = dart::dynamics::Group::create(name, mMetaSkeleton->getDofs());
  mStateSpace = std::make_shared<statespace::dart::MetaSkeletonStateSpace>(
      controlledMetaSkeleton.get());

  auto skeletonObj = getRootSkeleton();
  skeletonObj->enableSelfCollisionCheck();
  skeletonObj->disableAdjacentBodyCheck();

  setTrajectoryExecutor(trajExecutor);
  auto rngSeed = std::chrono::system_clock::now().time_since_epoch().count();
  mRng = std::make_unique<common::RNGWrapper<std::default_random_engine>>(
      rngSeed);
}

//==============================================================================
Robot::Robot(dart::dynamics::SkeletonPtr skeleton, const std::string name)
  : Robot(skeleton, name, internal::trajExecFromSkeleton(skeleton))
{
}

//==============================================================================
Robot::Robot(
    dart::dynamics::ReferentialSkeletonPtr refSkeleton,
    Robot* rootRobot,
    dart::collision::CollisionDetectorPtr collisionDetector,
    std::shared_ptr<dart::collision::BodyNodeCollisionFilter> collisionFilter,
    const std::string name)
  : mName(name)
  , mMetaSkeleton(refSkeleton)
  , mParentRobot(rootRobot)
  , mDofs(util::dofNamesFromSkeleton(refSkeleton))
  , mCollisionDetector(collisionDetector)
  , mSelfCollisionFilter(collisionFilter)
  , mWorld(nullptr)
{
  auto controlledMetaSkeleton
      = dart::dynamics::Group::create(name, mMetaSkeleton->getDofs());
  mStateSpace = std::make_shared<statespace::dart::MetaSkeletonStateSpace>(
      controlledMetaSkeleton.get());
}

//==============================================================================
void Robot::ignoreSelfCollisionPairs(
    const std::vector<std::pair<std::string, std::string>> bodyNodeList)
{
  for (auto pair : bodyNodeList)
  {
    auto body0 = mMetaSkeleton->getBodyNode(pair.first);
    auto body1 = mMetaSkeleton->getBodyNode(pair.second);
    if (body0 && body1)
    {
      mSelfCollisionFilter->addBodyNodePairToBlackList(body0, body1);
    }
  }
}

//==============================================================================
void Robot::enforceSelfCollisionPairs(
    const std::vector<std::pair<std::string, std::string>> bodyNodeList)
{
  for (auto pair : bodyNodeList)
  {
    auto body0 = mMetaSkeleton->getBodyNode(pair.first);
    auto body1 = mMetaSkeleton->getBodyNode(pair.second);
    if (body0 && body1)
    {
      mSelfCollisionFilter->removeBodyNodePairFromBlackList(body0, body1);
    }
  }
}

//==============================================================================
std::future<void> Robot::executeTrajectory(
    const trajectory::TrajectoryPtr& trajectory) const
{
  if (!mTrajectoryExecutor)
  {
    throw std::runtime_error(
        "TrajectoryExecutor is null, cannot execute trajectory.");
  }

  // Ensure all other executors that could contain these joints are cancelled
  if (mParentRobot)
  {
    mParentRobot->cancelAllTrajectories(false, true);
  }
  for (const auto& subrobot : mSubRobots)
  {
    subrobot.second->cancelAllTrajectories(true, false);
  }
  return mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
void Robot::cancelAllTrajectories(
    bool incluldeSubrobots,
    bool includeParents,
    const std::vector<std::string> excludedSubrobots)
{
  // Cancel this trajectory
  if (mTrajectoryExecutor)
  {
    mTrajectoryExecutor->cancel();
  }

  // Cancel parents' trajectories (if requested)
  if (includeParents && mParentRobot)
  {
    mParentRobot->cancelAllTrajectories(false, true);
  }

  // Cancel children's trajectories
  if (incluldeSubrobots)
  {
    for (const auto& subrobot : mSubRobots)
    {
      if (!std::count(
              excludedSubrobots.begin(),
              excludedSubrobots.end(),
              subrobot.first))
      {
        subrobot.second->cancelAllTrajectories(true, false);
      }
    }
  }
}

//==============================================================================
void Robot::step(const std::chrono::system_clock::time_point& timepoint)
{
  // Lock only if root robot
  std::unique_ptr<std::lock_guard<std::mutex>> lock;
  if (!mParentRobot)
  {
    lock = std::make_unique<std::lock_guard<std::mutex>>(
        getRootSkeleton()->getMutex());
  }

  if (mTrajectoryExecutor)
  {
    mTrajectoryExecutor->step(timepoint);
  }

  for (const auto& subrobot : mSubRobots)
  {
    subrobot.second->step(timepoint);
  }
}

// ==============================================================================
constraint::dart::CollisionFreePtr Robot::getSelfCollisionConstraint() const
{
  // Add collision option with self-collision filter
  auto collisionOption
      = dart::collision::CollisionOption(false, 1, mSelfCollisionFilter);

  auto collisionMetaSkeleton = mMetaSkeleton->cloneMetaSkeleton();

  // Create the constraint and return.
  auto collisionFreeConstraint
      = std::make_shared<constraint::dart::CollisionFree>(
          mStateSpace,
          collisionMetaSkeleton,
          mCollisionDetector,
          collisionOption);
  collisionFreeConstraint->addSelfCheck(
      mCollisionDetector->createCollisionGroupAsSharedPtr(
          collisionMetaSkeleton.get()));
  return collisionFreeConstraint;
}

// ==============================================================================
constraint::TestablePtr Robot::combineCollisionConstraint(
    const constraint::dart::CollisionFreePtr& collisionFree) const
{
  using constraint::TestableIntersection;

  auto selfCollisionFree = getSelfCollisionConstraint();

  if (!collisionFree)
    return selfCollisionFree;

  // Make testable constraints for collision check
  std::vector<aikido::constraint::ConstTestablePtr> constraints;
  constraints.reserve(2);
  constraints.emplace_back(selfCollisionFree);
  if (collisionFree)
  {
    if (collisionFree->getStateSpace() != mStateSpace)
    {
      dtwarn << "CollisionFreePtr space does not match robot space."
             << std::endl;
      return selfCollisionFree;
    }
    constraints.emplace_back(collisionFree);
  }

  return std::make_shared<TestableIntersection>(mStateSpace, constraints);
}

// ==============================================================================
constraint::TestablePtr Robot::getWorldCollisionConstraint(
    const std::vector<std::string> bodyNames) const
{
  auto world = getWorld();
  if (!world)
    return getSelfCollisionConstraint();

  std::shared_ptr<dart::collision::CollisionGroup> robotCollisionGroup
      = mCollisionDetector->createCollisionGroup(mMetaSkeleton.get());

  std::shared_ptr<dart::collision::CollisionGroup> worldCollisionGroup
      = mCollisionDetector->createCollisionGroup();
  for (std::string name : bodyNames)
  {
    auto skeleton = world->getSkeleton(name);
    if (skeleton)
    {
      worldCollisionGroup->addShapeFramesOf(skeleton.get());
    }
  }

  // Add all skeletons in world if bodyNames is empty
  if (bodyNames.size() == 0)
  {
    for (std::size_t i = 0; i < world->getNumSkeletons(); i++)
    {
      auto skeleton = world->getSkeleton(i);
      if (skeleton != getRootSkeleton())
      {
        worldCollisionGroup->addShapeFramesOf(skeleton.get());
      }
    }
  }

  // Create constraint and combine with self-constraint
  auto worldCollisionConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          mStateSpace, mMetaSkeleton, mCollisionDetector);
  worldCollisionConstraint->addPairwiseCheck(
      robotCollisionGroup, worldCollisionGroup);
  return combineCollisionConstraint(worldCollisionConstraint);
}

//=============================================================================
std::shared_ptr<Robot> Robot::registerSubRobot(
    dart::dynamics::ReferentialSkeletonPtr refSkeleton, const std::string& name)
{
  // Ensure name is unique
  if (mSubRobots.find(name) != mSubRobots.end())
  {
    dtwarn << "Subrobot '" << name << "' already exists." << std::endl;
    return nullptr;
  }

  // Ensure all body nodes in skeleton are owned by this robot
  for (auto bodyNode : refSkeleton->getBodyNodes())
  {
    if (!mMetaSkeleton->hasBodyNode(bodyNode))
    {
      dtwarn << "Subrobot '" << name << "'' contains body node "
             << bodyNode->getName() << " not in parent MetaSkeleton."
             << std::endl;
      return nullptr;
    }
  }

  // Ensure subrobot DoFs are disjoint
  for (const auto& subrobot : mSubRobots)
  {
    auto dofs = subrobot.second->mDofs;
    for (std::string dofName : util::dofNamesFromSkeleton(refSkeleton))
    {
      if (dofs.find(dofName) != dofs.end())
      {
        dtwarn << "Subrobot '" << name << "'' overlaps existing subrobot "
               << subrobot.first << " at DoF " << dofName << "." << std::endl;
        return nullptr;
      }
    }
  }

  // Create the subrobot.
  auto subRobot = std::make_shared<Robot>(
      refSkeleton, this, mCollisionDetector, mSelfCollisionFilter, name);
  mSubRobots[name] = subRobot;
  return subRobot;
}

//=============================================================================
Eigen::VectorXd Robot::getNamedConfiguration(const std::string& name) const
{
  auto configuration = mNamedConfigurations.find(name);
  if (configuration == mNamedConfigurations.end())
    return Eigen::VectorXd();

  return configuration->second;
}

//=============================================================================
void Robot::setNamedConfigurations(
    std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations)
{
  mNamedConfigurations = std::move(namedConfigurations);
}

//=============================================================================
trajectory::TrajectoryPtr Robot::planToConfiguration(
    const Eigen::VectorXd& goalConf,
    const constraint::TestablePtr& testableConstraint,
    const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
        planner,
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor) const
{
  auto goalState = mStateSpace->createState();
  try
  {
    mStateSpace->convertPositionsToState(goalConf, goalState);
  }
  catch (const std::exception& e)
  {
    dtwarn << "Cannot convert configuration of size " << goalConf.size()
           << " to robot state of size "
           << mStateSpace->getProperties().getNumDofs() << " : " << e.what()
           << std::endl;
    return nullptr;
  }

  // Create the problem.
  auto problem = planner::dart::ConfigurationToConfiguration(
      mStateSpace,
      mStateSpace->getScopedStateFromMetaSkeleton(mMetaSkeleton.get()),
      goalState,
      testableConstraint);

  // Default to base Snap Planner
  auto basePlanner = planner;
  if (!basePlanner)
  {
    using planner::SnapConfigurationToConfigurationPlanner;
    using statespace::GeodesicInterpolator;
    basePlanner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
        mStateSpace, std::make_shared<GeodesicInterpolator>(mStateSpace));
  }

  // Convert to DART Planner
  using planner::dart::
      ConfigurationToConfiguration_to_ConfigurationToConfiguration;
  auto dartPlanner = std::make_shared<
      ConfigurationToConfiguration_to_ConfigurationToConfiguration>(
      basePlanner, mMetaSkeleton);

  // Solve the problem with the DART planner.
  auto rawPlan = dartPlanner->plan(problem, /* result */ nullptr);

  // Postprocess if enabled or provided
  auto postprocessor
      = (trajPostProcessor)
            ? trajPostProcessor
            : ((mEnablePostProcessing) ? mDefaultPostProcessor : nullptr);
  if (rawPlan && postprocessor)
  {
    // Cast to interpolated or spline:
    auto interpolated
        = dynamic_cast<const aikido::trajectory::Interpolated*>(rawPlan.get());
    if (interpolated)
    {
      return postprocessor->postprocess(
          *interpolated, *(cloneRNG().get()), testableConstraint);
    }

    auto spline
        = dynamic_cast<const aikido::trajectory::Spline*>(rawPlan.get());
    if (spline)
    {
      return postprocessor->postprocess(
          *spline, *(cloneRNG().get()), testableConstraint);
    }

    // Else return raw path
  }
  return rawPlan;
}

//=============================================================================
trajectory::TrajectoryPtr Robot::planToConfiguration(
    const Eigen::VectorXd& goalConf,
    const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
        planner,
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor) const
{
  return planToConfiguration(
      goalConf, getSelfCollisionConstraint(), planner, trajPostProcessor);
}

//=============================================================================
trajectory::TrajectoryPtr Robot::planToOffset(
    const std::string bodyNodeName,
    const Eigen::Vector3d& offset,
    const constraint::TestablePtr& testableConstraint,
    const std::shared_ptr<
        planner::dart::ConfigurationToEndEffectorOffsetPlanner>& planner,
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor) const
{
  // Get Body Node
  auto bn = mMetaSkeleton->getBodyNode(bodyNodeName);
  if (!bn)
  {
    dtwarn << "Request body node not present in robot '" << mName << "'"
           << std::endl;
    return nullptr;
  }

  // Check offset
  auto distance = offset.norm();
  if (distance == 0)
    return nullptr;
  auto direction = offset / distance;

  // Create the problem.
  auto problem = planner::dart::ConfigurationToEndEffectorOffset(
      mStateSpace, mMetaSkeleton, bn, direction, distance, testableConstraint);

  // Default to VectorFieldParameter
  auto dartPlanner = planner;
  if (!dartPlanner)
  {
    using planner::vectorfield::
        VectorFieldConfigurationToEndEffectorOffsetPlanner;
    auto vfParams = util::VectorFieldPlannerParameters();
    dartPlanner
        = std::make_shared<VectorFieldConfigurationToEndEffectorOffsetPlanner>(
            mStateSpace,
            getMetaSkeletonClone(),
            vfParams.distanceTolerance,
            vfParams.positionTolerance,
            vfParams.angularTolerance,
            vfParams.initialStepSize,
            vfParams.jointLimitTolerance,
            vfParams.constraintCheckResolution,
            vfParams.timeout);
  }

  // Solve the problem with the DART planner.
  auto rawPlan = dartPlanner->plan(problem, /* result */ nullptr);

  // Postprocess if enabled or provided
  auto postprocessor
      = (trajPostProcessor)
            ? trajPostProcessor
            : ((mEnablePostProcessing) ? mDefaultPostProcessor : nullptr);
  if (rawPlan && postprocessor)
  {
    // Cast to interpolated or spline:
    auto interpolated
        = dynamic_cast<const aikido::trajectory::Interpolated*>(rawPlan.get());
    if (interpolated)
    {
      return postprocessor->postprocess(
          *interpolated, *(cloneRNG().get()), testableConstraint);
    }

    auto spline
        = dynamic_cast<const aikido::trajectory::Spline*>(rawPlan.get());
    if (spline)
    {
      return postprocessor->postprocess(
          *spline, *(cloneRNG().get()), testableConstraint);
    }

    // Else return raw path
  }
  return rawPlan;
}

//=============================================================================
trajectory::TrajectoryPtr Robot::planToOffset(
    const std::string bodyNodeName,
    const Eigen::Vector3d& offset,
    const std::shared_ptr<
        planner::dart::ConfigurationToEndEffectorOffsetPlanner>& planner,
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor) const
{
  return planToOffset(
      bodyNodeName,
      offset,
      getSelfCollisionConstraint(),
      planner,
      trajPostProcessor);
}

//=============================================================================
trajectory::TrajectoryPtr Robot::planToTSR(
    const std::string bodyNodeName,
    const constraint::dart::TSRPtr& tsr,
    const constraint::TestablePtr& testableConstraint,
    std::size_t maxSamplingTries,
    std::size_t batchSize,
    std::size_t maxBatches,
    const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
        planner,
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor,
    const distance::ConstConfigurationRankerPtr& ranker) const
{
  // Get Body Node
  auto bn = mMetaSkeleton->getBodyNode(bodyNodeName);
  if (!bn)
  {
    dtwarn << "Request body node not present in robot '" << mName << "'"
           << std::endl;
    return nullptr;
  }

  // Create the problem.
  auto problem = planner::dart::ConfigurationToTSR(
      mStateSpace,
      mStateSpace->getScopedStateFromMetaSkeleton(mMetaSkeleton.get()),
      bn,
      maxSamplingTries,
      batchSize,
      maxBatches,
      tsr,
      testableConstraint);

  // Default to base Snap Planner
  auto basePlanner = planner;
  if (!basePlanner)
  {
    using planner::SnapConfigurationToConfigurationPlanner;
    using statespace::GeodesicInterpolator;
    basePlanner = std::make_shared<SnapConfigurationToConfigurationPlanner>(
        mStateSpace, std::make_shared<GeodesicInterpolator>(mStateSpace));
  }

  // Convert to DART TSR planner.
  auto tsrPlanner = std::make_shared<
      planner::dart::ConfigurationToConfiguration_to_ConfigurationToTSR>(
      basePlanner, mMetaSkeleton, ranker);

  // Solve the problem with the DART planner.
  auto rawPlan = tsrPlanner->plan(problem, /* result */ nullptr);

  // Postprocess if enabled or provided
  auto postprocessor
      = (trajPostProcessor)
            ? trajPostProcessor
            : ((mEnablePostProcessing) ? mDefaultPostProcessor : nullptr);
  if (rawPlan && postprocessor)
  {
    // Cast to interpolated or spline:
    auto interpolated
        = dynamic_cast<const aikido::trajectory::Interpolated*>(rawPlan.get());
    if (interpolated)
    {
      return postprocessor->postprocess(
          *interpolated, *(cloneRNG().get()), testableConstraint);
    }

    auto spline
        = dynamic_cast<const aikido::trajectory::Spline*>(rawPlan.get());
    if (spline)
    {
      return postprocessor->postprocess(
          *spline, *(cloneRNG().get()), testableConstraint);
    }

    // Else return raw path
  }
  return rawPlan;
}

//=============================================================================
trajectory::TrajectoryPtr Robot::planToTSR(
    const std::string bodyNodeName,
    const constraint::dart::TSRPtr& tsr,
    std::size_t maxSamplingTries,
    std::size_t batchSize,
    std::size_t maxBatches,
    const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
        planner,
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor,
    const distance::ConstConfigurationRankerPtr& ranker) const
{
  return planToTSR(
      bodyNodeName,
      tsr,
      getSelfCollisionConstraint(),
      maxSamplingTries,
      batchSize,
      maxBatches,
      planner,
      trajPostProcessor,
      ranker);
}

//=============================================================================
std::string Robot::getName() const
{
  return mName;
}

//=============================================================================
dart::dynamics::MetaSkeletonPtr Robot::getMetaSkeletonClone() const
{
  return mMetaSkeleton->cloneMetaSkeleton();
}

//=============================================================================
dart::dynamics::MetaSkeletonPtr Robot::getMetaSkeleton()
{
  return mMetaSkeleton;
}

//=============================================================================
const dart::dynamics::MetaSkeletonPtr Robot::getMetaSkeleton() const
{
  return mMetaSkeleton;
}

//=============================================================================
Eigen::VectorXd Robot::getCurrentConfiguration() const
{
  return mMetaSkeleton->getPositions();
}

//=============================================================================
common::UniqueRNGPtr Robot::cloneRNG() const
{
  if (mRng)
  {
    return mRng->clone();
  }
  return nullptr;
}

//=============================================================================
void Robot::setRNG(common::UniqueRNGPtr rng)
{
  if (rng)
  {
    mRng = std::move(rng);
  }
}

//=============================================================================
aikido::planner::WorldPtr Robot::getWorld() const
{
  if (mParentRobot)
  {
    return mParentRobot->getWorld();
  }
  return mWorld;
}

//=============================================================================
void Robot::setWorld(aikido::planner::WorldPtr world)
{
  if (mParentRobot)
  {
    mParentRobot->setWorld(world);
    return;
  }

  mWorld = world;
  if (mWorld)
  {
    auto skeleton = getRootSkeleton();
    if (!mWorld->hasSkeleton(skeleton))
    {
      mWorld->addSkeleton(skeleton);
    }
  }
}

//=============================================================================
void Robot::setTrajectoryExecutor(
    const aikido::control::TrajectoryExecutorPtr& trajExecutor)
{
  if (mTrajectoryExecutor)
  {
    mTrajectoryExecutor->cancel();
  }
  mTrajectoryExecutor = trajExecutor;
}

//=============================================================================
void Robot::setDefaultPostProcessor(
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor)
{
  mDefaultPostProcessor = trajPostProcessor;
  mEnablePostProcessing = true;
}

//=============================================================================
std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
Robot::getDefaultPostProcessor() const
{
  return mDefaultPostProcessor;
}

//=============================================================================
void Robot::setEnablePostProcessing(bool enable)
{
  if (enable && !mDefaultPostProcessor)
  {
    // Initialize default post-processor
    mDefaultPostProcessor
        = std::make_shared<aikido::planner::kunzretimer::KunzRetimer>(
            mMetaSkeleton->getVelocityUpperLimits(),
            mMetaSkeleton->getAccelerationUpperLimits());
  }
  mEnablePostProcessing = enable;
}

//=============================================================================
dart::dynamics::ConstSkeletonPtr Robot::getRootSkeleton() const
{
  if (mParentRobot)
  {
    return mParentRobot->getRootSkeleton();
  }

  // Root robot should be a real skeleton
  return mMetaSkeleton->getBodyNode(0)->getSkeleton();
}

//=============================================================================
dart::dynamics::SkeletonPtr Robot::getRootSkeleton()
{
  if (mParentRobot)
  {
    return mParentRobot->getRootSkeleton();
  }

  // Root robot should be a real skeleton
  return mMetaSkeleton->getBodyNode(0)->getSkeleton();
}

} // namespace robot
} // namespace aikido

// TODO: Switch to PRIMITIVE once this is fixed in DART.
// mCollisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
