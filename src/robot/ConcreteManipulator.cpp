#include "aikido/robot/ConcreteManipulator.hpp"
#include "aikido/planner/dart/util.hpp"
#include "aikido/robot/util.hpp"

namespace aikido {
namespace robot {

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
std::unique_ptr<aikido::trajectory::Spline> ConcreteManipulator::retimePathWithKunzTimer(
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const aikido::trajectory::Trajectory* path)
{
  return mRobot->retimePathWithKunzTimer(metaSkeleton, path);
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

  auto collision
      = getFullCollisionConstraint(space, metaSkeleton, collisionFree);
  auto trajectory = util::planToEndEffectorOffset(
      space,
      metaSkeleton,
      body,
      direction,
      collision,
      distance,
      timelimit,
      positionTolerance,
      angularTolerance,
      mVectorFieldParameters,
      mCRRTParameters);

  return trajectory;
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
  auto collision
      = getFullCollisionConstraint(space, metaSkeleton, collisionFree);

  Eigen::Vector3d direction
      = planner::dart::util::getEndEffectorDirection(body);

  if (distance < 0)
  {
    distance = distance * -1;
    direction = direction * -1;
  }

  auto trajectory = util::planToEndEffectorOffset(
      space,
      metaSkeleton,
      body,
      direction,
      collision,
      distance,
      timelimit,
      positionTolerance,
      angularTolerance,
      mVectorFieldParameters,
      mCRRTParameters);

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
