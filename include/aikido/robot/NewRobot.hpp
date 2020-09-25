#ifndef AIKIDO_ROBOT_ROBOT_HPP_
#define AIKIDO_ROBOT_ROBOT_HPP_

#include <chrono>
#include <unordered_map>

#include <Eigen/Core>
#include <dart/dynamics/dynamics.hpp>

#include "aikido/common/RNG.hpp"
#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/constraint/dart/TSR.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Robot)

/// Robot interface for defining basic behaviors of a robot.
/// A concrete implementation of robot should support a collection
/// of planning methods.
struct Robot
{
public:
  Robot(
      const dart::common::Uri& urdf,
      const dart::common::Uri& srdf,
      const std::string& name = "robot");
  Robot(const MetaSkeletonPtr& skeleton, const std::string& name = "robot");
  virtual ~Robot() = default;

  // TODO(avk) : Requires access to action servers owning resources.
  std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) const;

  void step(const std::chrono::system_clock::time_point& timepoint);

  constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const;

  constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const constraint::dart::CollisionFreePtr& collisionFree) const;

  // Example planTo<> method.
  void planToConfiguration(Planner, GoalState, Constraint)
  {
    Problem problem(
        this->getStateSpace(), this->getCurrentState(), GoalState, Constraint);
    return Planner.plan(problem);
  }

  // Register child robots. Dispatching via factory.
  RobotPtr registerChildRobot(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::string& name,
      const RobotType type)
  {
    if (mChildren.find(name))
    {
      // Throw exception.
    }

    // Dispatch via factory.
    std::shared_ptr<Robot> child;
    if (type == RobotType::Manipulator)
    {
      child = std::make_shared<Manipulator>(metaSkeleton, name);
    }
    else if (type == RobotType::WheeledBase)
    {
      child = std::make_shared<WheeledBase>(metaSkeleton, name);
    }
    const auto& root = mRootRobot ? mRootRobot : get_shared_from_this();
    child->setRootRobot(root);
    mChildren[name] = child;
    return child;
  }

  std::string mName;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  std::shared_ptr<Robot> mRootRobot{nullptr};
  std::unordered_map<std::string, RobotPtr> mChildren;
  const RobotType mType;
};

// Example Manipulator class to handle hands.
class Manipulator : public Robot
{
  Manipulator(urdf, srdf, name);
  Manipulator(metaSkeleton, name);
  registerChildRobot(metaskeleton, name, type) override
  {
    auto child = Robot::registerChildRobot;
    if (type == RobotType::Gripper)
    {
      mHasHand = true;
      mHand = child;
    }
  }

  void planToEndEffectorOffset(Planner, Direction, Distance)
  {
    if (!hasHand())
    {
      // Throw exception.
    }
    // Create problem.
    // Plan.
  }

}

// ============================================================================
class HERB : public Robot
{
  constexpr std::string leftArmName = "leftarm";
  constexpr std::string rightArmName = "rightarm";
  HERB(urdf, srdf) : Robot(urdf, srdf)
  {
    // Register left arm.
    auto leftarm = registerChildRobot(
        LeftMetaSkeletonPtr, leftArmName, RobotType::Manipulator);
    leftarm->registerChildRobot(
        LeftHandSkeletonPtr, "lefthand", RobotType::Gripper);

    // Register right arm.
    auto rightarm = registerChildRobot(
        RightMetaSkeletonPtr, rightArmName, RobotType::Manipulator);
    rightarm->registerChildRobot(
        RightHandSkeletonPtr, "righthand", RobotType::Gripper);
  }
}

// ============================================================================
int main()
{
  // Construct the robot.
  HERB herb(urdf, srdf, "herb");

  // Provide the action servers associated.
  herb.setTrajectoryExecutors(serverNames);

  // Move the left arm.
  auto trajectory
      = herb.getLeftArm()->planToConfiguration(planner, goal, constraint);
  herb.executeTrajectory();

  // Close the right hand.
  herb.getRightArm()->closeHand();
}

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_ROBOT_HPP_
