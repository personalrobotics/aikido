#include <aikido/robot/Robot.hpp>

namespace aikido {
namespace robot {

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToConfiguration(
    const statespace::StateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    double timelimit)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  return mPlanner->planToConfiguration(
    stateSpace, metaSkeleton, startState, goalState, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const Eigen::VectorXd &goal,
    const CollisionFreePtr &collisionFree,
    double timelimit)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto startState = stateSpace.createState();
  stateSpace->convertPositionsToState(start, startState);

  auto goalState = stateSpace.createState();
  stateSpace->convertPositionsToState(goal, goalState);

  return planToConfiguration(stateSpace, metaSkeleton, startState, goalState, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr planToConfiguration(
    const Configuration &configuration,
    const CollisionFreePtr &collisionFree,
    double timelimit)
{
  auto stateSpace = configuration.metaSkeletonStateSpcae;
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto startState = stateSpace.createState();
  stateSpace->convertPositionsToState(configuration.metaSkeleton.getPositions(),
    startState);

  auto goalState = stateSpace.createState();
  stateSpace->convertPositionsToState(configuration.positions, goalState);

  return planToConfiguration(stateSpace, configuration.metaSkeleton,
    startState, goalState, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToConfigurations(
    const statespace::StateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State* startState,
    const std::vector<statespace::StateSpace::State*> goalStates,
    double timelimit)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  return mPlanner->planToConfigurations(
    statespace, metaSkeleton, startState, goalStates, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const std::vector<Eigen::VectorXd> &goals,
    const CollisionFreePtr &collisionFree,
    double timelimit)
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
    startState, goalStates, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToTSR(
    const statespace::StateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const statespace::StateSpace::State* startState,
    const aikido::constraint::TSR& tsr,
    double timelimit)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  return mPlanner->planToTSR(statespace, metaSkeleton,
    startState, tsr, timelimit);
}

//==============================================================================
 aikido::trajectory::TrajectoryPtr Robot::planToTSR(
    const statespace::MetaSkeletonStateSpacePtr& stateSpace,
    const MetaSkeletonPtr &metaSkeleton,
    const Eigen::VectorXd &start,
    const aikido::constraint::TSR &tsr,
    const CollisionFreePtr &collisionFree,
    double timelimit)
{
  if (!checkIfMetaSkeletonBelongs(metaSkeleton))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto startState = stateSpace.createState();
  stateSpace->convertPositionsToState(start, startState);

  return planToTSR(stateSpace, metaSkeleton,
    startState, tsr, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr planToNamedConfiguration(
    const std::string &name,
    const CollisionFreePtr &collisionFree,
    double timelimit)
{
  if (mNamedConfigurations.find(name) == mNamedConfigurations.end())
    throw std::runtime_error(name + " does not exist.");

  auto configuration = mNamedConfigurations[name];
  planToConfiguration(configuration, metaSkeleton,
    collisionFree, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::postprocessPath(
  const aikido::trajectory::TrajectoryPtr &path,
  const CollisionFreePtr &collisionFree,
  double timelimit)
{
  throw std::runtime_error("Not implemented");
}

//==============================================================================
void Robot::executeTrajectory(
    const aikido::trajectory::TrajectoryPtr &trajectory)
{

}

//==============================================================================
void Robot::executePath(
    const aikido::trajectory::TrajectoryPtr &path,
    const CollisionFreePtr &collisionFree,
    double timelimit)
{
  auto traj = postprocessPath(path, collisionFree, timelimit);
  executeTrajectory(traj);
}


//==============================================================================
bool Robot::checkIfMetaSkeletonBelongs(
  const MetaSkeletonPtr &metaSkeleton,)
{
  // TODO: check if the robot can handle statespace.
  throw std::runtime_error("Not implemented");
}

//==============================================================================
bool Robot::switchControllers(
    const std::vector<std::string>& start_controllers,
    const std::vector<std::string>& stop_controllers)
{
  if (!mNode)
    throw std::runtime_error("Ros node has not been instantiated.");

  if (!mControllerServiceClient)
    throw std::runtime_error("ServiceClient not instantiated.");

  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers = start_controllers;
  srv.request.stop_controllers = stop_controllers;
  srv.request.strictness
      = controller_manager_msgs::SwitchControllerRequest::STRICT;

  if (mControllerServiceClient->call(srv))
    return srv.response.ok;
  else
    throw std::runtime_error("SwitchController failed.");
}

//==============================================================================
void Robot::setConfiguration(
    const std::string &name,
    const Eigen::VectorXd &configuration)
{
  if (mMetaSkeletons.find(name) == mMetaSkeletons.end())
    throw std::runtime_error(name + " does not exist.");

  auto metaSkeleton = mMetaSkeletons[name].first;

  metaSkeleton->setPositions(configuration);
}

//==============================================================================
Eigen::VectorXd Robot::getConfiguration(
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
aikido::planner::WorldPtr Robot::getParentWorld()
{
  return mWorld;
}

//==============================================================================
std::string Robot::getName()
{
  return mName;
}

//==============================================================================
dart::dynamics::SkeletonPtr Robot::getRobot()
{
  return mRobot;
}

//==============================================================================
std::pair<dart::dynamics::MetaSkeletonPtr,
      aikido::statespace::dart::MetaSkeletonStateSpacePtr>>
  Robot::getMetaSkeletonStateSpacePair(
    const std::string& name)
{
  return mMetaSkeletons[name];
}

// ==============================================================================
CollisionFreePtr Robot::getSelfCollisionConstraint(
    const MetaSkeletonStateSpacePtr &space) const
{
  using aikido::constraint::CollisionFree;

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
TestablePtr Robot::getTestableCollisionConstraint(
    const MetaSkeletonStateSpacePtr &space,
    const CollisionFreePtr &collisionFree) const
{
  using aikido::constraint::TestableIntersection;

  auto selfCollisionFree = getSelfCollisionConstraint(space);

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

} // namespace robot
} // namespace aikido
