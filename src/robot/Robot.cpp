#include "aikido/robot/Robot.hpp"

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/common/Console.hpp>

#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/planner/SnapConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToConfiguration.hpp"
#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToConfiguration.hpp"
#include "aikido/planner/dart/ConfigurationToConfiguration_to_ConfigurationToTSR.hpp"
#include "aikido/planner/dart/ConfigurationToTSR.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace robot {

//==============================================================================
Robot::Robot(
    const dart::dynamics::MetaSkeletonPtr& skeleton,
    const std::string name,
    const aikido::control::TrajectoryExecutorPtr& trajExecutor)
  : mName(name)
  , mMetaSkeleton(skeleton)
  , mDofs(util::dofNamesFromSkeleton(skeleton))
  , mCollisionDetector(dart::collision::FCLCollisionDetector::create())
{
  mStateSpace = std::make_shared<statespace::dart::MetaSkeletonStateSpace>(
      mMetaSkeleton.get());
  auto skeletonObj = mMetaSkeleton->getBodyNode(0)->getSkeleton();
  skeletonObj->enableSelfCollisionCheck();
  skeletonObj->disableAdjacentBodyCheck();

  setTrajectoryExecutor(trajExecutor);
  auto rngSeed = std::chrono::system_clock::now().time_since_epoch().count();
  mRng = std::make_unique<common::RNGWrapper<std::default_random_engine>>(
      rngSeed);
  mWorld = std::make_shared<aikido::planner::World>();
  mWorld->addSkeleton(skeletonObj);
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
std::future<void> Robot::executeTrajectory(
    const trajectory::TrajectoryPtr& trajectory) const
{
  if (!mTrajectoryExecutor)
  {
    throw std::runtime_error(
        "TrajectoryExecutor is null, cannot execute trajectory.");
  }

  // Ensure all other executors that could contain these joints are cancelled
  if (mRootRobot)
  {
    mRootRobot->cancelAllTrajectories(true, getName());
  }
  for (const auto& subrobot : mSubRobots)
  {
    subrobot.second->cancelAllTrajectories(false);
  }
  return mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
void Robot::cancelAllTrajectories(
    bool includeParents, const std::string excludeSubrobot)
{
  // Cancel this trajectory
  if (mTrajectoryExecutor)
  {
    mTrajectoryExecutor->cancel();
  }

  // Cancel parents' trajectories (if requested)
  if (includeParents && mRootRobot)
  {
    mRootRobot->cancelAllTrajectories(true, getName());
  }

  // Cancel children's trajectories (unless requested)
  for (const auto& subrobot : mSubRobots)
  {
    if (subrobot.first != excludeSubrobot)
    {
      subrobot.second->cancelAllTrajectories(false);
    }
  }
}

//==============================================================================
void Robot::step(const std::chrono::system_clock::time_point& timepoint)
{
  // TODO(avk): Is the following statement valid? Where is it conveyed to the
  // user and what are its implications and where is the parent robot locked?
  // Assumes that the parent robot is locked
  mTrajectoryExecutor->step(timepoint);
}

// ==============================================================================
constraint::dart::CollisionFreePtr Robot::getSelfCollisionConstraint() const
{
  // Only use root robot's self-collision constraint
  if (mRootRobot)
  {
    return mRootRobot->getSelfCollisionConstraint();
  }

  // Add collision option with self-collision filter
  auto collisionOption
      = dart::collision::CollisionOption(false, 1, mSelfCollisionFilter);

  // Create the constraint and return.
  auto collisionFreeConstraint
      = std::make_shared<constraint::dart::CollisionFree>(
          mStateSpace,
          mMetaSkeleton->cloneMetaSkeleton(),
          mCollisionDetector,
          collisionOption);
  collisionFreeConstraint->addSelfCheck(
      mCollisionDetector->createCollisionGroupAsSharedPtr(mMetaSkeleton.get()));
  return collisionFreeConstraint;
}

// ==============================================================================
constraint::TestablePtr Robot::combineCollisionConstraint(
    const constraint::dart::CollisionFreePtr& collisionFree) const
{
  using constraint::TestableIntersection;

  // Only use root robot's self-collision constraint
  if (mRootRobot)
  {
    return mRootRobot->combineCollisionConstraint(collisionFree);
  }

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
  // Only use root robot's self-collision constraint and World
  if (mRootRobot)
  {
    return mRootRobot->getWorldCollisionConstraint(bodyNames);
  }

  if (!mWorld)
    return getSelfCollisionConstraint();

  auto robotCollisionGroup
      = mCollisionDetector->createCollisionGroup(mMetaSkeleton.get());

  auto worldCollisionGroup = mCollisionDetector->createCollisionGroup();
  for (std::string name : bodyNames)
  {
    auto skeleton = mWorld->getSkeleton(name);
    if (skeleton)
    {
      worldCollisionGroup->addShapeFramesOf(skeleton.get());
    }
  }

  // Add all skeletons in world if bodyNames is empty
  if (bodyNames.size() == 0)
  {
    for (std::size_t i = 0; i < mWorld->getNumSkeletons(); i++)
    {
      auto skeleton = mWorld->getSkeleton(i);
      if (skeleton != mMetaSkeleton->getBodyNode(0)->getSkeleton())
      {
        worldCollisionGroup->addShapeFramesOf(skeleton.get());
      }
    }
  }

  // Create constraint and combine with self-constraint
  auto worldCollisionConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          mStateSpace, mMetaSkeleton, mCollisionDetector);
  return combineCollisionConstraint(worldCollisionConstraint);
}

//=============================================================================
std::shared_ptr<Robot> Robot::registerSubRobot(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::string& name)
{
  // Ensure name is unique
  if (mSubRobots.find(name) != mSubRobots.end())
  {
    dtwarn << "Subrobot '" << name << "' already exists." << std::endl;
    return nullptr;
  }

  // Ensure subrobot DoFs are disjoint
  for (const auto& subrobot : mSubRobots)
  {
    auto dofs = subrobot.second->mDofs;
    for (std::string dofName : util::dofNamesFromSkeleton(metaSkeleton))
    {
      if (dofs.find(dofName) != dofs.end())
      {
        dtwarn << "Subrobot '" << name << "'' overlaps existing subrobot "
               << subrobot.first << "." << std::endl;
        return nullptr;
      }
    }
  }

  // Create the subrobot.
  auto subRobot = std::make_shared<Robot>(metaSkeleton, name);
  const auto& root = mRootRobot ? mRootRobot : RobotPtr(this);
  subRobot->setRootRobot(root);
  subRobot->setWorld(nullptr); // inherit root robot world
  mSubRobots[name] = subRobot;
  return subRobot;
}

//=============================================================================
trajectory::TrajectoryPtr Robot::planToConfiguration(
    const statespace::StateSpace::State* goalState,
    const constraint::TestablePtr& testableConstraint,
    const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
        planner) const
{
  // Create the problem.
  auto problem = planner::dart::ConfigurationToConfiguration(
      mStateSpace,
      mStateSpace->cloneState(getCurrentState()),
      mStateSpace->cloneState(goalState),
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

  // Postprocess if enabled
  if (rawPlan && mEnablePostProcessing && mDefaultPostProcessor)
  {
    // Cast to interpolated or spline:
    auto interpolated
        = dynamic_cast<const aikido::trajectory::Interpolated*>(rawPlan.get());
    if (interpolated)
    {
      return mDefaultPostProcessor->postprocess(
          *interpolated, *(cloneRNG().get()), testableConstraint);
    }

    auto spline
        = dynamic_cast<const aikido::trajectory::Spline*>(rawPlan.get());
    if (spline)
    {
      return mDefaultPostProcessor->postprocess(
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
    const constraint::TestablePtr& testableConstraint,
    std::size_t maxSamples,
    const std::shared_ptr<planner::ConfigurationToConfigurationPlanner>&
        planner,
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
      mStateSpace->cloneState(getCurrentState()),
      bn,
      maxSamples,
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

  // Postprocess if enabled
  if (rawPlan && mEnablePostProcessing && mDefaultPostProcessor)
  {
    // Cast to interpolated or spline:
    auto interpolated
        = dynamic_cast<const aikido::trajectory::Interpolated*>(rawPlan.get());
    if (interpolated)
    {
      return mDefaultPostProcessor->postprocess(
          *interpolated, *(cloneRNG().get()), testableConstraint);
    }

    auto spline
        = dynamic_cast<const aikido::trajectory::Spline*>(rawPlan.get());
    if (spline)
    {
      return mDefaultPostProcessor->postprocess(
          *spline, *(cloneRNG().get()), testableConstraint);
    }

    // Else return raw path
  }
  return rawPlan;
}

} // namespace robot
} // namespace aikido

// TODO: Switch to PRIMITIVE once this is fixed in DART.
// mCollisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
