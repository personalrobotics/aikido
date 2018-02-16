#include "aikido/robot/Robot.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"
#include "aikido/constraint/TestableIntersection.hpp"

namespace aikido {
namespace robot {

using common::RNG;
using constraint::CollisionFreePtr;
using constraint::TSR;
using constraint::TSRPtr;
using constraint::TestablePtr;
using statespace::dart::MetaSkeletonStateSpacePtr;
using statespace::dart::MetaSkeletonStateSaver;
using statespace::StateSpace;
using statespace::StateSpacePtr;
using trajectory::TrajectoryPtr;
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


// TODO: Temporary constants for planning calls.
// These should be defined when we construct planner adapter classes
static const double timelimit = 3.0;
static const double maxNumTrials = 10;
static const double collisionResolution = 0.1;

//==============================================================================
Robot::Robot(
  const std::string& name,
  MetaSkeletonPtr robot,
  MetaSkeletonStateSpacePtr statespace,
  bool simulation,
  aikido::common::RNG::result_type rngSeed,
  std::unique_ptr<control::TrajectoryExecutor> trajectoryExecutor,
  std::unique_ptr<planner::parabolic::ParabolicTimer> retimer,
  std::unique_ptr<planner::parabolic::ParabolicSmoother> smoother)
: mRootRobot(this)
, mName(name)
, mRobot(robot)
, mStateSpace(statespace)
, mParentRobot(mRobot->getBodyNode(0)->getSkeleton())
, mSimulation(simulation)
, mRng(rngSeed)
, mTrajectoryExecutor(std::move(trajectoryExecutor))
, mRetimer(std::move(retimer))
, mSmoother(std::move(smoother))
, mCollisionResolution(collisionResolution)
{
  if (!mSimulation)
    throw std::invalid_argument("Not implemented");

  mCollisionDetector = FCLCollisionDetector::create();
  mCollideWith = mCollisionDetector->createCollisionGroupAsSharedPtr();
  mSelfCollisionFilter
      = std::make_shared<dart::collision::BodyNodeCollisionFilter>();
  mSelfCollisionConstraint = createSelfCollisionConstraint();
}

//==============================================================================
TrajectoryPtr Robot::planToConfiguration(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const StateSpace::State* goalState,
    const CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto collisionConstraint = getCollisionConstraint(stateSpace, collisionFree);

  return util::planToConfiguration(
    stateSpace, metaSkeleton, goalState,
    collisionConstraint, cloneRNG().get());
}

//==============================================================================
TrajectoryPtr Robot::planToConfiguration(
    const MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &goal,
    const CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto goalState = stateSpace->createState();
  stateSpace->convertPositionsToState(goal, goalState);

  return planToConfiguration(stateSpace, metaSkeleton,
    goalState, collisionFree);
}

//==============================================================================
TrajectoryPtr Robot::planToConfigurations(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const std::vector<StateSpace::State*> &goalStates,
    const CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  return util::planToConfigurations(
    stateSpace, metaSkeleton, goalStates,
    collisionFree, cloneRNG().get());
}

//==============================================================================
TrajectoryPtr Robot::planToConfigurations(
    const MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const std::vector<Eigen::VectorXd> &goals,
    const CollisionFreePtr &collisionFree)
{

  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  std::vector<StateSpace::State*> goalStates;
  for(const auto goal: goals)
  {
    auto goalState = stateSpace->createState();
    stateSpace->convertPositionsToState(goal, goalState);
  }

  return planToConfigurations(stateSpace, metaSkeleton,
    goalStates, collisionFree);
}

//==============================================================================
TrajectoryPtr Robot::planToTSR(
    const MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const dart::dynamics::BodyNodePtr& bn,
    const TSRPtr& tsr,
    const CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto collisionConstraint = getCollisionConstraint(stateSpace, collisionFree);

  // Todo: ignoring startstate
  return util::planToTSR(stateSpace, metaSkeleton,
    bn, tsr, collisionConstraint, cloneRNG().get());
}

//==============================================================================
TrajectoryPtr Robot::
  planToTSRwithTrajectoryConstraint(
      const MetaSkeletonStateSpacePtr &stateSpace,
      const MetaSkeletonPtr &metaSkeleton,
      const BodyNodePtr &bodyNode,
      const TSRPtr &goalTsr,
      const TSRPtr &constraintTsr,
      const CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto collisionConstraint = getCollisionConstraint(stateSpace, collisionFree);

  return util::planToTSRwithTrajectoryConstraint(
    stateSpace,
    metaSkeleton,
    bodyNode,
    goalTsr,
    constraintTsr,
    collisionConstraint);
}

//==============================================================================
TrajectoryPtr Robot::planToNamedConfiguration(
    const std::string &name,
    const CollisionFreePtr &collisionFree)
{
  if (mNamedConfigurations.find(name) == mNamedConfigurations.end())
    throw std::runtime_error(name + " does not exist.");

  auto configuration = mNamedConfigurations[name];
  auto goalState = configuration.metaSkeletonStateSpace->createState();
  auto stateSpace = configuration.metaSkeletonStateSpace;
  stateSpace->convertPositionsToState(configuration.position, goalState);

  return planToConfiguration(
    stateSpace,
    configuration.metaSkeleton,
    goalState, collisionFree);
}

//==============================================================================
TrajectoryPtr Robot::postprocessPath(
  const TrajectoryPtr &path)
{

  // TODO: this needs to know whether to call smoothing or not.
  std::unique_ptr<aikido::trajectory::Spline> untimedTrajectory;

  auto spline = std::dynamic_pointer_cast<trajectory::Spline>(path);
  if (spline)
    auto untimedTrajectory =  mSmoother->postprocess(*(spline.get()), *(cloneRNG().get()));

  auto interpolated = std::dynamic_pointer_cast<trajectory::Interpolated>(path);
  if (interpolated)
    untimedTrajectory =  mSmoother->postprocess(*(interpolated.get()), *(cloneRNG().get()));

  if (!untimedTrajectory)
    throw std::invalid_argument("_inputTraj must be either Spline or Interpolated");

  auto timedTrajectory = mRetimer->postprocess(
      *(untimedTrajectory.get()), *(cloneRNG().get()));

  return std::move(timedTrajectory);

}

//==============================================================================
void Robot::executeTrajectory(
    const TrajectoryPtr &trajectory)
{
  mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
void Robot::executePath(const TrajectoryPtr &path)
{
  auto traj = postprocessPath(path);
  executeTrajectory(traj);
}

//==============================================================================
Robot::Configuration Robot::getNamedConfiguration(
    const std::string& name)
{
  if (mNamedConfigurations.find(name) == mNamedConfigurations.end())
    throw std::runtime_error(name + " does not exist.");

  return mNamedConfigurations[name];
}

//==============================================================================
Eigen::VectorXd Robot::getConfiguration()
{
  return mRobot->getPositions();
}

//==============================================================================
std::string Robot::getName()
{
  return mName;
}

//==============================================================================
MetaSkeletonPtr Robot::getMetaSkeleton()
{
  return mRobot;
}

//==============================================================================
void Robot::step(const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mParentRobot->getMutex());
  mTrajectoryExecutor->step(timepoint);
}

//==============================================================================
void Robot::update()
{
  step(std::chrono::system_clock::now());
}

//==============================================================================
std::unique_ptr<common::RNG> Robot::cloneRNG()
{
  return std::move(cloneRNGFrom(mRng)[0]);
}

//==============================================================================
bool Robot::checkIfMetaSkeletonBelongs(
  const MetaSkeletonPtr &metaSkeleton)
{
  // TODO: check if the robot can handle statespace.
  throw std::runtime_error("Not implemented");
}

// ==============================================================================
CollisionFreePtr Robot::getSelfCollisionConstraint() const
{
  return mSelfCollisionConstraint;
}

// ==============================================================================
CollisionFreePtr Robot::createSelfCollisionConstraint() const
{
  using constraint::CollisionFree;

  if (mRootRobot != this)
    return mRootRobot->getSelfCollisionConstraint();

  mParentRobot->enableSelfCollisionCheck();
  mParentRobot->disableAdjacentBodyCheck();

  auto collisionDetector = FCLCollisionDetector::create();

  // TODO: Switch to PRIMITIVE once this is fixed in DART.
  // collisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  auto collisionOption
      = dart::collision::CollisionOption(false, 1, mSelfCollisionFilter);
  auto collisionFreeConstraint = std::make_shared<CollisionFree>(
      mStateSpace, mRobot, collisionDetector, collisionOption);
  collisionFreeConstraint->addSelfCheck(
      collisionDetector->createCollisionGroupAsSharedPtr(mRobot.get()));
  return collisionFreeConstraint;
}

//=============================================================================
TestablePtr Robot::getCollisionConstraint(
    const MetaSkeletonStateSpacePtr &space,
    const CollisionFreePtr &collisionFree) const
{
  using constraint::TestableIntersection;

  if (mRootRobot != this)
    return mRootRobot->getCollisionConstraint(space, collisionFree);

  auto selfCollisionFree = getSelfCollisionConstraint();

  // Make testable constraints for collision check
  std::vector<TestablePtr> constraints;
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

//=============================================================================
void Robot::setRoot(Robot *robot)
{
  if (robot == nullptr)
    throw std::invalid_argument("Robot is null.");

  mRootRobot = robot;
}

} // namespace robot
} // namespace aikido

