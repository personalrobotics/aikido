#include <aikido/robot/Robot.hpp>

namespace aikido {
namespace robot {

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToConfiguration(
    const statespace::StateSpacePtr& stateSpace,
    const statespace::StateSpace::State* startState,
    const statespace::StateSpace::State* goalState,
    double timeLimit)
{
  if (!checkIfStateSpaceBelongs(stateSpace))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  return mPlanner->planToConfiguration(
    stateSpace, startState, goalState, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToConfiguration(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const Eigen::VectorXd &start,
    const Eigen::VectorXd &goal,
    const CollisionFreePtr &collisionFree,
    double timeLimit)
{
  if (!checkIfStateSpaceBelongs(stateSpace))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto startState = stateSpace.createState();
  stateSpace->convertPositionsToState(start, startState);

  auto goalState = stateSpace.createState();
  stateSpace->convertPositionsToState(goal, goalState);

  return planToConfiguration(stateSpace, startState, goalState, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToConfigurations(
    const statespace::StateSpacePtr& stateSpace,
    const statespace::StateSpace::State* startState,
    const std::vector<statespace::StateSpace::State*> goalStates,
    double timeLimit)
{
  if (!checkIfStateSpaceBelongs(stateSpace))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  return mPlanner->planToConfigurations(
    statespace, startState, goalStates, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToConfigurations(
    const statespace::MetaSkeletonStateSpacePtr &stateSpace,
    const Eigen::VectorXd &start,
    const std::vector<Eigen::VectorXd> &goals,
    const CollisionFreePtr &collisionFree,
    double timeLimit)
{
  using statespace::StateSpace::State;

  if (!checkIfStateSpaceBelongs(stateSpace))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto startState = stateSpace.createState();
  stateSpace->convertPositionsToState(start, startState);

  std::vector<State*> goalStates;
  for(const auto goal: goals)
  {
    auto goalState = stateSpace.createState();
    stateSpace->convertPositionsToState(goal, goalStates);
  }

  return planToConfigurations(stateSpace, startState, goalStates, timelimit);
}

//==============================================================================
aikido::trajectory::TrajectoryPtr Robot::planToTSR(
    const statespace::StateSpacePtr& stateSpace,
    const statespace::StateSpace::State* startState,
    const aikido::constraint::TSR& tsr,
    double timelimit)
{
  if (!checkIfStateSpaceBelongs(stateSpace))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  return mPlanner->planToTSR(statespace, startState, tsr, timelimit);
}

//==============================================================================
 aikido::trajectory::TrajectoryPtr Robot::planToTSR(
    const statespace::MetaSkeletonStateSpacePtr& stateSpace,
    const Eigen::VectorXd &start,
    const aikido::constraint::TSR &tsr,
    const CollisionFreePtr &collisionFree,
    double timelimit)
{
  if (!checkIfStateSpaceBelongs(stateSpace))
    throw std::runtime_error("Statespace is incompatible with this robot.");

  auto startState = stateSpace.createState();
  stateSpace->convertPositionsToState(start, startState);

  return planToTSR(stateSpace, startState, tsr, timeLimit);
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
bool Robot::checkIfStateSpaceBelongs(
  const statespace::MetaSkeletonStateSpacePtr& stateSpace)
{
  for(const auto& space : mStateSpaces)
  {
    if (stateSpace == space)
      return true;
  }

  return false;
}

} // namespace robot
} // namespace aikido
