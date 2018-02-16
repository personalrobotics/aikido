#include "aikido/robot/Manipulator.hpp"
#include "aikido/robot/util.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSaver.hpp"

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
    std::shared_ptr<Hand> hand)
  : Robot(
        name,
        robot,
        statespace,
        simulation,
        rngSeed,
        std::move(executor),
        std::move(retimer),
        std::move(smoother))
  , mHand(hand)
{
  if (!mHand)
    throw std::invalid_argument("Hand is null");
}

//==============================================================================
TrajectoryPtr Manipulator::planToEndEffectorOffset(
    const MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const BodyNodePtr& body,
    const Eigen::Vector3d& direction,
    const CollisionFreePtr& collisionFree,
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
    const dart::dynamics::BodyNodePtr& body) const
{
  const size_t zDirection = 2;
  return body->getWorldTransform().linear().col(zDirection).normalized();
}

//==============================================================================
TrajectoryPtr Manipulator::planEndEffectorStraight(
    MetaSkeletonStateSpacePtr& space,
    const MetaSkeletonPtr& metaSkeleton,
    const BodyNodePtr& body,
    const CollisionFreePtr& collisionFree,
    double distance,
    double linearVelocity)
{
  auto collision = getCollisionConstraint(space, collisionFree);

  Eigen::Vector3d direction = getEndEffectorDirection(body);

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
      linearVelocity);
  return trajectory;
}

//==============================================================================
std::shared_ptr<Hand> Manipulator::getHand()
{
  return mHand;
}

//==============================================================================
void Manipulator::step(const std::chrono::system_clock::time_point& timepoint)
{
  std::lock_guard<std::mutex> lock(mParentRobot->getMutex());
  mHand->step(timepoint);
  mTrajectoryExecutor->step(timepoint);
}
}
}
