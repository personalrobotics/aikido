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
#include "aikido/planner/vectorfield/VectorFieldConfigurationToEndEffectorPosePlanner.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace robot {

//==============================================================================
Robot::Robot(
    dart::dynamics::SkeletonPtr skeleton,
    const std::string name,
    bool addDefaultExecutors)
  : mName(name)
  , mMetaSkeleton(skeleton)
  , mDofs(util::dofNamesFromSkeleton(skeleton))
  , mCollisionDetector(dart::collision::FCLCollisionDetector::create())
  , mSelfCollisionFilter(
        std::make_shared<dart::collision::BodyNodeCollisionFilter>())
{
  mStateSpace = std::make_shared<statespace::dart::MetaSkeletonStateSpace>(
      mMetaSkeleton.get());

  auto skeletonObj = getRootSkeleton();
  skeletonObj->enableSelfCollisionCheck();
  skeletonObj->disableAdjacentBodyCheck();

  auto rngSeed = std::chrono::system_clock::now().time_since_epoch().count();
  mRng = std::make_unique<common::RNGWrapper<std::default_random_engine>>(
      rngSeed);

  // Add default executors if requested
  if (addDefaultExecutors)
  {
    auto trajExecName = registerExecutor(
        std::make_shared<
            aikido::control::KinematicSimulationTrajectoryExecutor>(skeleton));
    activateExecutor(trajExecName);

    registerExecutor(
        std::make_shared<aikido::control::KinematicSimulationPositionExecutor>(
            skeleton));
    registerExecutor(
        std::make_shared<aikido::control::KinematicSimulationVelocityExecutor>(
            skeleton));
  }
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
  mStateSpace = std::make_shared<statespace::dart::MetaSkeletonStateSpace>(
      mMetaSkeleton.get());
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
void Robot::clearExecutors()
{
  deactivateExecutor();
  mExecutors.clear();
}

//==============================================================================
void Robot::deactivateExecutor()
{
  if (mActiveExecutor >= 0)
  {
    mExecutors[mActiveExecutor]->releaseDofs();
    mActiveExecutor = -1;
  }
}

//==============================================================================
aikido::control::ExecutorPtr Robot::getActiveExecutor()
{
  if (mActiveExecutor >= 0)
  {
    return mExecutors[mActiveExecutor];
  }
  return nullptr;
}

//==============================================================================
int Robot::registerExecutor(aikido::control::ExecutorPtr executor, std::string desiredName)
{
  if (!executor)
  {
    return -1;
  }

  executor->releaseDofs();
  mExecutors.push_back(executor);         

  if(!desiredName.empty())
  {
    if(mExecutorsNameMap.find(desiredName) != mExecutorsNameMap.end() && mActiveExecutor == mExecutorsNameMap[desiredName])
    {
      deactivateExecutor();
    }
    mExecutorsNameMap[desiredName] = mExecutors.size() - 1;
  }

  return mExecutors.size() - 1;
}

//==============================================================================
bool Robot::activateExecutor(const int id)
{
  // Deactivate active executor
  deactivateExecutor();

  // Validate input
  if (id < 0 || (size_t)id >= mExecutors.size())
  {
    dtwarn << "Could not activate executor as id is invalid."
       << std::endl;
    return false;
  }

  // If we can register the executor, activate it
  if (mExecutors[id]->registerDofs())
  {
    mActiveExecutor = (int)id;
    return true;
  }
  return false;
}

//==============================================================================
bool Robot::activateExecutor(const std::string name)
{
  // Validate input
  if (mExecutorsNameMap.find(name) == mExecutorsNameMap.end())
  {
    return false;
  }

  return activateExecutor(mExecutorsNameMap[name]);
}

//==============================================================================
bool Robot::activateExecutor(const aikido::control::ExecutorType type)
{
  // Search for last added executor of given type
  for (int i = mExecutors.size() - 1; i >= 0; i--)
  {
    auto types = mExecutors[i]->getTypes();
    if (types.find(type) != types.end())
    {
      return activateExecutor(i);
    }
  }

  // Deactivate active executor
  deactivateExecutor();
  return false;
}

//==============================================================================
// Explicit Instantiation of Joint Commnd Functions
template std::future<int>
Robot::executeJointCommand<aikido::control::ExecutorType::POSITION>(
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout);
template std::future<int>
Robot::executeJointCommand<aikido::control::ExecutorType::VELOCITY>(
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout);
template std::future<int>
Robot::executeJointCommand<aikido::control::ExecutorType::EFFORT>(
    const std::vector<double>& command,
    const std::chrono::duration<double>& timeout);

//==============================================================================
std::future<void> Robot::executeTrajectory(
    const trajectory::TrajectoryPtr& trajectory)
{
  // Retrieve active executor
  if (mActiveExecutor < 0)
  {
    return common::make_exceptional_future<void>(
        "executeTrajectory: No active executor");
  }
  aikido::control::TrajectoryExecutorPtr trajectoryExecutor;
  trajectoryExecutor
      = std::dynamic_pointer_cast<aikido::control::TrajectoryExecutor>(
          mExecutors[mActiveExecutor]);
  if (!trajectoryExecutor)
  {
    return common::make_exceptional_future<void>(
        "executeTrajectory: Active executor not a TrajectoryExecutor");
  }

  return trajectoryExecutor->execute(trajectory);
}

//==============================================================================
void Robot::cancelAllCommands(
    bool includeSubrobots,
    bool includeParents,
    const std::vector<std::string> excludedSubrobots)
{
  // Cancel this trajectory
  if (mActiveExecutor >= 0)
  {
    mExecutors[mActiveExecutor]->cancel();
  }

  // Cancel parents' trajectories (if requested)
  if (includeParents && mParentRobot)
  {
    mParentRobot->cancelAllCommands(false, true);
  }

  // Cancel children's trajectories (if requested)
  if (includeSubrobots)
  {
    for (const auto& subrobot : mSubRobots)
    {
      if (!std::count(
              excludedSubrobots.begin(),
              excludedSubrobots.end(),
              subrobot.first))
      {
        subrobot.second->cancelAllCommands(true, false);
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

  if (mActiveExecutor >= 0)
  {
    mExecutors[mActiveExecutor]->step(timepoint);
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

  // Create the constraint and return.
  auto collisionFreeConstraint
      = std::make_shared<constraint::dart::CollisionFree>(
          mStateSpace, mMetaSkeleton, mCollisionDetector, collisionOption);
  collisionFreeConstraint->addSelfCheck(
      mCollisionDetector->createCollisionGroupAsSharedPtr(
          getRootSkeleton().get()));
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
bool Robot::validateSubRobot(
    dart::dynamics::ReferentialSkeletonPtr refSkeleton, const std::string& name)
{
  // Ensure name is unique
  if (mSubRobots.find(name) != mSubRobots.end())
  {
    dtwarn << "Subrobot '" << name << "' already exists." << std::endl;
    return false;
  }

  // Ensure all body nodes in skeleton are owned by this robot
  for (auto bodyNode : refSkeleton->getBodyNodes())
  {
    if (!mMetaSkeleton->hasBodyNode(bodyNode))
    {
      dtwarn << "Subrobot '" << name << "'' contains body node "
             << bodyNode->getName() << " not in parent MetaSkeleton."
             << std::endl;
      return false;
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
        return false;
      }
    }
  }

  return true;
}

//=============================================================================
std::shared_ptr<Robot> Robot::registerSubRobot(
    dart::dynamics::ReferentialSkeletonPtr refSkeleton, const std::string& name)
{
  if (!validateSubRobot(refSkeleton, name))
  {
    return nullptr;
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
  auto bn = getRootSkeleton()->getBodyNode(bodyNodeName);
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
            getMetaSkeleton(),
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
trajectory::TrajectoryPtr Robot::planToPoseOffset(
    const std::string bodyNodeName,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& rotation,
    const constraint::TestablePtr& testableConstraint,
    const std::shared_ptr<planner::dart::ConfigurationToEndEffectorPosePlanner>&
        planner,
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor) const
{
  // Get Body Node
  auto bn = getRootSkeleton()->getBodyNode(bodyNodeName);
  if (!bn)
  {
    dtwarn << "Request body node not present in robot '" << mName << "'"
           << std::endl;
    return nullptr;
  }

  // Check angle
  auto angle = rotation.norm();
  if (angle == 0)
  {
    dtwarn << "Zero Rotation. Use PlanToOffset instead." << std::endl;
  }
  auto axis = (angle == 0) ? rotation : rotation / angle;

  // Create the problem.
  auto startPose = bn->getTransform();
  auto goalPose = Eigen::Isometry3d::Identity();
  goalPose.translation() = startPose.translation() + offset;
  goalPose.linear() = Eigen::AngleAxisd(angle, axis) * startPose.linear();
  auto problem = planner::dart::ConfigurationToEndEffectorPose(
      mStateSpace, mMetaSkeleton, bn, goalPose, testableConstraint);

  // Default to VectorFieldParameter
  auto dartPlanner = planner;
  std::shared_ptr<aikido::planner::Planner::Result> result;
  if (!dartPlanner)
  {
    using planner::vectorfield::
        VectorFieldConfigurationToEndEffectorPosePlanner;
    auto vfParams = util::VectorFieldPlannerParameters();
    dartPlanner
        = std::make_shared<VectorFieldConfigurationToEndEffectorPosePlanner>(
            mStateSpace,
            getMetaSkeleton(),
            vfParams.goalTolerance,
            vfParams.angleDistanceRatio,
            vfParams.positionTolerance,
            vfParams.angularTolerance,
            vfParams.initialStepSize,
            vfParams.jointLimitTolerance,
            vfParams.constraintCheckResolution,
            vfParams.timeout);
    result = std::make_shared<
        VectorFieldConfigurationToEndEffectorPosePlanner::Result>();
  }

  // Solve the problem with the DART planner.
  auto rawPlan = dartPlanner->plan(problem, result.get());
  if (!rawPlan && result)
    dtwarn << result->getMessage() << std::endl;

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
trajectory::TrajectoryPtr Robot::planToPoseOffset(
    const std::string bodyNodeName,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& rotation,
    const std::shared_ptr<planner::dart::ConfigurationToEndEffectorPosePlanner>&
        planner,
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor) const
{
  return planToPoseOffset(
      bodyNodeName,
      offset,
      rotation,
      getSelfCollisionConstraint(),
      planner,
      trajPostProcessor);
}

//=============================================================================
trajectory::TrajectoryPtr Robot::planToTSR(
    const std::string bodyNodeName,
    const constraint::dart::TSRPtr& tsr,
    const constraint::TestablePtr& testableConstraint,
    const util::PlanToTSRParameters& params,
    const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
        planner,
    const std::shared_ptr<aikido::planner::TrajectoryPostProcessor>
        trajPostProcessor,
    const distance::ConstConfigurationRankerPtr& ranker) const
{
  // Get Body Node
  auto bn = getRootSkeleton()->getBodyNode(bodyNodeName);
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
      params.maxSamplingTries,
      params.batchSize,
      params.maxBatches,
      params.numMaxIterations,
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
    const util::PlanToTSRParameters& params,
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
      params,
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
