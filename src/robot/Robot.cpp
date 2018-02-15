#include "aikido/robot/Robot.hpp"
#include "aikido/robot/util.hpp"

namespace aikido {
namespace robot {

// TODO: Temporary constants for planning calls.
// These should be defined when we construct planner adapter classes
static const double timelimit = 3.0;
static const double maxNumTrials = 10;

//==============================================================================
Robot::Robot(
  const std::string& name,
  MetaSkeletonPtr robot,
  MetaSkeletonStateSpacePtr statespace,
  bool simulation,
  aikido::common::RNG::result_type rngSeed,
  std::unique_ptr<control::TrajectoryExecutor> trajectoryExecutor,
  std::unique_ptr<planner::parabolic::ParabolicTimer> retimer,
  std::unique_ptr<planner::parabolic::ParabolicSmoother> smoother,
  double collisionResolution)
: mRootRobot(this)
: mName(name)
, mRobot(robot)
, mStateSpace(statespace)
, mSimulation(simulation)
, mRng(rngSeed)
, mTrajectoryExecutor(std::move(trajectoryExecutor))
, mRetimer(std::move(retimer))
, mSmoother(std::move(smoother))
, mCollisionResolution(collisionResolution)
{
  if (!mSimulation)
    throw std::error("Not implemented");

  mCollisionDetector = FCLCollisionDetector::create();
  mCollideWith = mCollisionDetector->createCollisionGroupAsSharedPtr();
  mSelfCollisionFilter
      = std::make_shared<dart::collision::BodyNodeCollisionFilter>();
  mSelfCollisionConstraint = createSelfCollisionConstraint();
}

//==============================================================================
Robot::~Robot()
{
  // Do nothing
}

//==============================================================================
trajectory::TrajectoryPtr Robot::planToConfiguration(
    const statespace::StateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    const constraint::CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto collisionConstraint = getCollisionConstraint(stateSpace, collisionFree);

  return util::planToConfiguration(
    stateSpace, metaSkeleton, startState, goalState, timelimit,
    collisionConstraint, cloneRNG(), mCollisionResolution);
}

//==============================================================================
trajectory::TrajectoryPtr Robot::planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const Eigen::VectorXd &goal,
    const CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto startState = stateSpace.createState();
  stateSpace->convertPositionsToState(start, startState);

  auto goalState = stateSpace.createState();
  stateSpace->convertPositionsToState(goal, goalState);

  return planToConfiguration(stateSpace, metaSkeleton,
    startState, goalState, collisionFree);
}

//==============================================================================
trajectory::TrajectoryPtr Robot::planToConfigurations(
    const statespace::StateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State* startState,
    const std::vector<statespace::StateSpace::State*> goalStates,
    const CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  return planToConfigurations(
    statespace, metaSkeleton, startState, goalStates, timelimit,
    collisionFree, cloneRNG(), mCollisionResolution);
}

//==============================================================================
trajectory::TrajectoryPtr Robot::planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const std::vector<Eigen::VectorXd> &goals,
    const CollisionFreePtr &collisionFree)
{
  using statespace::StateSpace::State;

  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto startState = stateSpace.createState();
  stateSpace->convertPositionsToState(start, startState);

  std::vector<State*> goalStates;
  for(const auto goal: goals)
  {
    auto goalState = stateSpace.createState();
    stateSpace->convertPositionsToState(goal, goalStates);
  }

  return planToConfigurations(stateSpace, metaSkeleton,
    startState, goalStates, collisioNFree);
}

//==============================================================================
trajectory::TrajectoryPtr Robot::planToTSR(
    const statespace::StateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State* startState,
    const constraint::TSR& tsr,
    const constraint::CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto collisionConstraint = getCollisionConstraint(stateSpace, collisionFree);

  return util::planToTSR(statespace, metaSkeleton,
    startState, tsr, maxNumTrials, timelimit, collisionConstraint, cloneRNG(),
    mCollisionResolution);
}

//==============================================================================
 trajectory::TrajectoryPtr Robot::planToTSR(
    const statespace::MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const constraint::TSR &tsr,
    const CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto startState = stateSpace.createState();
  stateSpace->convertPositionsToState(start, startState);

  return planToTSR(stateSpace, metaSkeleton,
    startState, tsr, collisionFree);
}

//==============================================================================
trajectory::TrajectoryPtr Robot::
  planToTSRwithTrajectoryConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &bodyNode,
      const constraint::TSRPtr &goalTsr,
      const constraint::TSRPtr &constraintTsr,
      const CollisionFreePtr &collisionFree)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto collisionConstraint = getCollisionConstraint(stateSpace, collisionFree);

  util::planToTSRwithTrajectoryConstraint(
    space,
    metaSkeleton,
    bodyNode,
    goalTsr,
    constraintTsr,
    collisionConstraint);
}

//==============================================================================
trajectory::TrajectoryPtr Robot::planToNamedConfiguration(
    const statespace::StateSpace::State* startState,
    const std::string &name,
    const CollisionFreePtr &collisionFree)
{
  if (mNamedConfigurations.find(name) == mNamedConfigurations.end())
    throw std::runtime_error(name + " does not exist.");

  auto goalState = stateSpace.createState();
  stateSpace->convertPositionsToState(goal, goalState);

  auto configuration = mNamedConfigurations[name];
  planToConfiguration(configuration.metaSkeletonStateSpace,
    configuration.metaSkeleton,
    startState, goalState, collisionFree);
}

//==============================================================================
trajectory::TrajectoryPtr Robot::postprocessPath(
  const trajectory::TrajectoryPtr &path)
{
  // TODO: this needs to know whether to call smoothing or not.
  auto untimedTrajectory = mSmoother->postprocess(path, cloneRNG());
  auto timedTrajectory = mRetimer->postprocess(untimedTrajectory, cloneRNG());

  return timedTrajectory;
}

//==============================================================================
void Robot::executeTrajectory(
    const trajectory::TrajectoryPtr &trajectory)
{
  mTrajectoryExecutor->execute(trajectory);
}

//==============================================================================
void Robot::executePath(const trajectory::TrajectoryPtr &path)
{
  auto traj = postprocessPath(path, timelimit);
  executeTrajectory(traj);
}

//==============================================================================
Eigen::VectorXd Robot::getNamedConfiguration(
    const std::string& name)
{
  if (mMetaSkeletons.find(name) == mMetaSkeletons.end())
    throw std::runtime_error(name + " does not exist.");

  auto metaSkeleton = mMetaSkeletons[name].first;

  return metaSkeleton->getPositions();
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
  std::lock_guard<std::mutex> lock(mRobot->getMutex());
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

  mRobot->enableSelfCollisionCheck();
  mRobot->disableAdjacentBodyCheck();

  auto collisionDetector = FCLCollisionDetector::create();

  // TODO: Switch to PRIMITIVE once this is fixed in DART.
  // collisionDetector->setPrimitiveShapeType(FCLCollisionDetector::PRIMITIVE);
  auto collisionOption
      = dart::collision::CollisionOption(false, 1, mSelfCollisionFilter);
  auto collisionFreeConstraint = std::make_shared<CollisionFree>(
      space, collisionDetector, collisionOption);
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

