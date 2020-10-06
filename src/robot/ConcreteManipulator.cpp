#include "aikido/robot/ConcreteManipulator.hpp"

#include "aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp"
#include "aikido/planner/dart/util.hpp"
#include "aikido/planner/vectorfield/VectorFieldConfigurationToEndEffectorOffsetPlanner.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"

namespace aikido {
namespace robot {

using planner::dart::ConfigurationToEndEffectorOffset;
using planner::vectorfield::VectorFieldConfigurationToEndEffectorOffsetPlanner;
using statespace::dart::MetaSkeletonStateSaver;

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
  // NOTE: We plan using VFP.
  auto vfpPlanner
      = std::make_shared<VectorFieldConfigurationToEndEffectorOffsetPlanner>(
          space,
          metaSkeleton,
          mVectorFieldParameters.distanceTolerance,
          positionTolerance,
          angularTolerance,
          mVectorFieldParameters.initialStepSize,
          mVectorFieldParameters.jointLimitTolerance,
          mVectorFieldParameters.constraintCheckResolution,
          std::chrono::duration<double>(timelimit));

  // Create planning problem.
  auto collisionConstraint
      = getFullCollisionConstraint(space, metaSkeleton, collisionFree);
  auto problem = ConfigurationToEndEffectorOffset(
      space, metaSkeleton, body, direction, distance, collisionConstraint);

  // Plan.
  return vfpPlanner->plan(problem);
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

  return planToEndEffectorOffset(
      space,
      metaSkeleton,
      body,
      collisionFree,
      direction,
      distance,
      timelimit,
      positionTolerance,
      angularTolerance);
}

//==============================================================================
void ConcreteManipulator::setVectorFieldPlannerParameters(
    const util::VectorFieldPlannerParameters& vfParameters)
{
  mVectorFieldParameters = vfParameters;
}

} // namespace robot
} // namespace aikido
