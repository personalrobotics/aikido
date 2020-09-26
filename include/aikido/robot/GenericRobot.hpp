#ifndef AIKIDO_ROBOT_GENERICROBOT_HPP_
#define AIKIDO_ROBOT_GENERICROBOT_HPP_

#include <chrono>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/robot/Hand.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(GenericRobot)

enum class RobotType
{
  GenericRobot,
  Manipulator,
  WheeledBase,
  Gripper
};

/// Robot interface for defining basic behaviors of a robot.
/// A concrete implementation of robot should support a collection
/// of planning methods.
/// TODO(avk): Derive from enable_shared_from_this<GenericRobot>.
class GenericRobot
{
public:
  GenericRobot(
      const dart::common::Uri& urdf,
      const dart::common::Uri& srdf,
      const std::string& name = "robot");
  GenericRobot(
      const dart::dynamics::MetaSkeletonPtr& skeleton,
      const std::string& name = "robot");
  virtual ~GenericRobot() = default;

  virtual std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) const;

  virtual void step(const std::chrono::system_clock::time_point& timepoint);

  constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const;

  constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const constraint::dart::CollisionFreePtr& collisionFree) const;

  // Register child robots. Dispatching via factory.
  virtual std::shared_ptr<GenericRobot> registerChildRobot(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::string& name,
      const RobotType type);

  virtual RobotType getRobotType() const
  {
    return RobotType::GenericRobot;
  }

  void setRootRobot(std::shared_ptr<GenericRobot> root)
  {
    mRootRobot = root;
  }

  std::string mName;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  std::shared_ptr<control::TrajectoryExecutor> mTrajectoryExecutor;
  std::shared_ptr<GenericRobot> mRootRobot{nullptr};
  std::unordered_map<std::string, std::shared_ptr<GenericRobot>> mChildren;
};

// Example Manipulator class to handle hands.
class GenericGripper : public GenericRobot
{
public:
  GenericGripper(
      const dart::common::Uri& urdf,
      const dart::common::Uri& srdf,
      const std::string& name = "manipulator")
    : GenericRobot(urdf, srdf, name)
  {
  }

  GenericGripper(
      const dart::dynamics::MetaSkeletonPtr& skeleton,
      const std::string& name = "manipulator")
    : GenericRobot(skeleton, name)
  {
  }

  std::shared_ptr<GenericRobot> registerChildRobot(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::string& name,
      const RobotType type) override
  {
    // TODO(avk): Not implemented.
    return nullptr;
  }

  RobotType getRobotType() const override
  {
    return RobotType::Gripper;
  }
};

// Example Manipulator class to handle hands.
class GenericManipulator : public GenericRobot
{
public:
  GenericManipulator(
      const dart::common::Uri& urdf,
      const dart::common::Uri& srdf,
      const std::string& name = "manipulator")
    : GenericRobot(urdf, srdf, name)
  {
  }

  GenericManipulator(
      const dart::dynamics::MetaSkeletonPtr& skeleton,
      const std::string& name = "manipulator")
    : GenericRobot(skeleton, name)
  {
  }

  std::shared_ptr<GenericRobot> registerChildRobot(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::string& name,
      const RobotType type) override;

  RobotType getRobotType() const override
  {
    return RobotType::Manipulator;
  }

private:
  std::shared_ptr<GenericGripper> mGripper{nullptr};
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_GENERICROBOT_HPP_