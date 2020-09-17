#include "aikido/robot/Manipulator.hpp"

namespace aikido {
namespace robot {

//==============================================================================
Manipulator::Manipulator(
    const std::string& name,
    dart::dynamics::MetaSkeletonPtr metaSkeleton,
    HandPtr hand,
    aikido::common::UniqueRNGPtr rng,
    aikido::control::TrajectoryExecutorPtr trajectoryExecutor,
    dart::collision::CollisionDetectorPtr collisionDetector,
    std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
        selfCollisionFilter)
  : Robot(
      name,
      std::move(metaSkeleton),
      std::move(rng),
      std::move(trajectoryExecutor),
      std::move(collisionDetector),
      std::move(selfCollisionFilter))
  , mHand(std::move(hand))
{
  // Do nothing
}

//==============================================================================
ConstHandPtr Manipulator::getHand() const
{
  return mHand;
}

//==============================================================================
HandPtr Manipulator::getHand()
{
  return std::const_pointer_cast<Hand>(
      const_cast<const Manipulator*>(this)->getHand());
}

//==============================================================================
void Manipulator::step(const std::chrono::system_clock::time_point& timepoint)
{
  mHand->step(timepoint);
  Robot::step(timepoint);
}

} // namespace robot
} // namespace aikido
