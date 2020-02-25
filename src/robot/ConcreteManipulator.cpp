#include "aikido/robot/ConcreteManipulator.hpp"

#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/planner/dart/ConfigurationToTSRwithTrajectoryConstraint.hpp"
#include "aikido/planner/dart/CRRTConfigurationToTSRwithTrajectoryConstraintPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToTSRPlanner.hpp"
#include "aikido/planner/dart/util.hpp"
#include "aikido/planner/vectorfield/VectorFieldPlanner.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"

namespace aikido {
namespace robot {

using planner::dart::ConfigurationToTSRwithTrajectoryConstraint;
using planner::dart::CRRTConfigurationToTSRwithTrajectoryConstraintPlanner;
using statespace::dart::MetaSkeletonStateSaver;
using constraint::dart::TSR;

//==============================================================================
ConcreteManipulator::ConcreteManipulator(RobotPtr robot, HandPtr hand)
  : mRobot(robot), mHand(hand)
{
  // Do nothing
}

//==============================================================================
ConstHandPtr ConcreteManipulator::getHand() const
{
  return mHand;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> ConcreteManipulator::smoothPath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path,
    const constraint::TestablePtr& constraint)
{
  return mRobot->smoothPath(metaSkeleton, path, constraint);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> ConcreteManipulator::retimePath(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path)
{
  return mRobot->retimePath(metaSkeleton, path);
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline>
ConcreteManipulator::retimePathWithKunz(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path,
    double maxDeviation,
    double timestep)
{
  return mRobot->retimePathWithKunz(metaSkeleton, path, maxDeviation, timestep);
}

//==============================================================================
std::future<void> ConcreteManipulator::executeTrajectory(
    const trajectory::TrajectoryPtr& trajectory) const
{
  return mRobot->executeTrajectory(trajectory);
}

//==============================================================================
boost::optional<Eigen::VectorXd> ConcreteManipulator::getNamedConfiguration(
    const std::string& name) const
{
  return mRobot->getNamedConfiguration(name);
}

//==============================================================================
void ConcreteManipulator::setNamedConfigurations(
    std::unordered_map<std::string, const Eigen::VectorXd> namedConfigurations)
{
  mRobot->setNamedConfigurations(namedConfigurations);
}

//==============================================================================
std::string ConcreteManipulator::getName() const
{
  return mRobot->getName();
}

//==============================================================================
dart::dynamics::ConstMetaSkeletonPtr ConcreteManipulator::getMetaSkeleton()
    const
{
  return mRobot->getMetaSkeleton();
}

//==============================================================================
statespace::dart::ConstMetaSkeletonStateSpacePtr
ConcreteManipulator::getStateSpace() const
{
  return mRobot->getStateSpace();
}

//==============================================================================
void ConcreteManipulator::setRoot(Robot* robot)
{
  mRobot->setRoot(robot);
}

//==============================================================================
void ConcreteManipulator::step(
    const std::chrono::system_clock::time_point& timepoint)
{
  mHand->step(timepoint);
  mRobot->step(timepoint);
}

//==============================================================================
constraint::dart::CollisionFreePtr
ConcreteManipulator::getSelfCollisionConstraint(
    const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const
{
  return mRobot->getSelfCollisionConstraint(space, metaSkeleton);
}

//==============================================================================
aikido::constraint::TestablePtr ConcreteManipulator::getFullCollisionConstraint(
    const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const constraint::dart::CollisionFreePtr& collisionFree) const
{
  return mRobot->getFullCollisionConstraint(space, metaSkeleton, collisionFree);
}

//==============================================================================
trajectory::TrajectoryPtr ConcreteManipulator::planToEndEffectorOffset(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& body,
    const constraint::dart::CollisionFreePtr& collisionFree,
    const Eigen::Vector3d& direction,
    double distance,
    double timelimit,
    double positionTolerance,
    double angularTolerance)
{
  auto saver = MetaSkeletonStateSaver(metaSkeleton);
  DART_UNUSED(saver);

  auto collisionTestable
      = getFullCollisionConstraint(space, metaSkeleton, collisionFree);

  auto startState = space->createState();
  space->getState(metaSkeleton.get(), startState);

  auto minDistance
      = std::max(0.0, distance - mVectorFieldParameters.negativeDistanceTolerance);
  auto maxDistance = distance + mVectorFieldParameters.positiveDistanceTolerance;

  auto traj = planner::vectorfield::planToEndEffectorOffset(
      space,
      *startState,
      metaSkeleton,
      body,
      collisionTestable,
      direction,
      minDistance,
      maxDistance,
      positionTolerance,
      angularTolerance,
      mVectorFieldParameters.initialStepSize,
      mVectorFieldParameters.jointLimitTolerance,
      mVectorFieldParameters.constraintCheckResolution,
      std::chrono::duration<double>(timelimit));

  if (traj)
    return std::move(traj);

  return planToEndEffectorOffsetByCRRT(
      space,
      metaSkeleton,
      body,
      collisionTestable,
      direction,
      distance,
      timelimit,
      positionTolerance,
      angularTolerance,
      mCRRTParameters);
}

//==============================================================================
trajectory::TrajectoryPtr ConcreteManipulator::planToEndEffectorOffsetByCRRT(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const constraint::TestablePtr& collisionTestable,
    const Eigen::Vector3d& direction,
    double distance,
    double timelimit,
    double positionTolerance,
    double angularTolerance,
    const util::CRRTPlannerParameters& crrtParameters)
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
  bool success = util::getGoalAndConstraintTSRForEndEffectorOffset(
      bodyNode,
      directionCopy,
      distance,
      goalTsr,
      constraintTsr,
      positionTolerance,
      angularTolerance);

  if (!success)
    throw std::runtime_error("failed in creating TSR");

  // Create problem
  auto problem = ConfigurationToTSRwithTrajectoryConstraint(
    space, metaSkeleton,
    bodyNode, goalTsr, constraintTsr, collisionTestable);

  // Create planner
  auto planner = std::make_shared<CRRTConfigurationToTSRwithTrajectoryConstraintPlanner>(
    space, metaSkeleton, timelimit, crrtParameters);

  CRRTConfigurationToTSRwithTrajectoryConstraintPlanner::Result pResult;

  return planner->plan(problem, &pResult);
}

//==============================================================================
trajectory::TrajectoryPtr ConcreteManipulator::planEndEffectorStraight(
    statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& body,
    const constraint::dart::CollisionFreePtr& collisionFree,
    double distance,
    double timelimit,
    double positionTolerance,
    double angularTolerance)
{
  Eigen::Vector3d direction
      = planner::dart::util::getEndEffectorDirection(body);

  if (distance < 0)
  {
    distance = distance * -1;
    direction = direction * -1;
  }

  auto trajectory = planToEndEffectorOffset(
      space,
      metaSkeleton,
      body,
      collisionFree,
      direction,
      distance,
      timelimit,
      positionTolerance,
      angularTolerance);

  return trajectory;
}

//==============================================================================
void ConcreteManipulator::setVectorFieldPlannerParameters(
    const util::VectorFieldPlannerParameters& vfParameters)
{
  mVectorFieldParameters = vfParameters;
}

//=============================================================================
void ConcreteManipulator::setCRRTPlannerParameters(
    const util::CRRTPlannerParameters& crrtParameters)
{
  mCRRTParameters = crrtParameters;
}

} // namespace robot
} // namespace aikido
