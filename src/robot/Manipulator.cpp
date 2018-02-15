#include "aikido/robot/Manipulator.hpp"
#include "aikido/robot/util.hpp"

namespace aikido {
namespace robot {

//==============================================================================
Manipulator::Manipulator(
      const std::string& name,
      MetaSkeletonPtr robot,
      MetaSkeletonStateSpacePtr statespace,
      bool simulation,
      common::RNG::result_type rngSeed,
      std::unique_ptr<control::TrajectoryExecutor> executor,
      std::unique_ptr<planner::parabolic::ParabolicTimer> retimer,
      std::unique_ptr<planner::parabolic::ParabolicSmoother> smoother,
      double collisionResolution,
      std::unique_ptr<Hand> hand)
  : Robot(name,
    robot,
    statespace,
    simulation,
    rngSeed,
    executor,
    retimer,
    collisionResolution)
  , mHand(std::move(hand))
{
  if (!mHand)
    throw std::invalid_argument("Hand is null");
}

//==============================================================================
Manipulator::~Manipulator()
{
  // Do nothing
}

//==============================================================================
trajectory::TrajectoryPtr Manipulator::planToEndEffectorOffset(
  const statespace::dart::MetaSkeletonStateSpacePtr &space,
   const dart::dynamics::MetaSkeletonptr &metaSkeleton,
   const dart::dynamics::BodyNodePtr &body,
   const Eigen::Vector3d &direction,
   const constraint::CollisionFreePtr &collisionFree,
   double distance,
   double linearVelocity)
{
  auto collision = getCollisionConstraint(space, collisionFree);
  auto trajectory = util::planToEndEffectorOffset(
    space,
    metaSkeleton,
    body,
    direction,
    collision,
    distance,
    linearVelocity);

  return trajectory;
}

//==============================================================================
Eigen::Vector3d Manipulator::getEndEffectorDirection(
      const dart::dynamics::BodyNodePtr &body)
{
  const size_t zDirection = 2;
  return body->getWorldTransform().linear().col(zDirection).normalized();

}

//==============================================================================
trajectory::TrajectoryPtr Manipulator::planEndEffectorStraight(
      statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &body,
      const constraint::CollisionFreePtr &collisionFree,
      double distance,
      double linearVelocity)
{
  auto collision = getCollisionConstraint(space, collisionFree);
  auto trajectory = util::planEndEffectorStraight(
    space,
    metaSkeleton,
    body,
    collision,
    distance,
    linearVelocity);
}

//==============================================================================
std::shared_ptr<Hand> Manipulator::getHand()
{
  return mHand;
}

//==============================================================================
void Manipulator::step(const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mRobot->getMutex());
  mHand->step(timepoint);
  mTrajectoryExecutor->step(timepoint);
}

}
}
