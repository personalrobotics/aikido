#ifndef AIKIDO_ROBOT_ROBOT_HPP_
#define AIKIDO_ROBOT_ROBOT_HPP_

// TODO(avk): How many of these headers are actually needed?
#include <chrono>
#include <string>
#include <unordered_map>

#include <Eigen/Core>
#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionFilter.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include "aikido/constraint/dart/CollisionFree.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/planner/Planner.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"
#include <aikido/statespace/StateSpace.hpp>

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Robot)

/// Robot interface for defining basic behaviors of a robot.
/// A concrete implementation of robot should support a collection
/// of planning methods.
class Robot
{
public:
  ///
  /// Construct a new Robot object.
  ///
  /// \param[in] urdf The path to the robot's URDF.
  /// \param[in] srdf The path to the robot's SRDF.
  /// \param[in] name The name of the robot.
  // TODO(avk): Default resource retriever and executor to nullptr.
  Robot(
      const dart::common::Uri& urdf,
      const dart::common::Uri& srdf,
      const std::string name = "robot");

  ///
  /// Construct a new Robot object.
  ///
  /// \param[in] skeleton The skeleton defining the robot.
  /// \param[in] name The name of the robot.
  Robot(
      const dart::dynamics::MetaSkeletonPtr& skeleton,
      const std::string& name = "robot");

  /// Destructor.
  virtual ~Robot() = default;

  ///
  /// Executes given trajectory on the robot.
  ///
  /// \param[in] trajectory The trajectory to execute.
  virtual std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) const;

  ///
  /// Steps the robot through time.
  ///
  /// \param[in] timepoint The point in time to step to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint);

  // TODO(avk): If I use mSpace and mSkeleton, the whole robot moves around
  // during planning. Is it a locking issue?
  constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      statespace::dart::MetaSkeletonStateSpacePtr space,
      dart::dynamics::MetaSkeletonPtr skeleton) const;

  ///
  /// Given a collision constraint, returns it coupled with the self collision
  /// constraint of the entire robot.
  /// TODO(avk): Are the following parameters really necessary?
  /// \param[in] space The statespace the constraint is specified in.
  /// \param[in] skeleton The skeleton the constraint is specified for.
  /// \param[in] collisionFree The collision constraint that is to be tied with
  /// parent's self collision-constraint.
  constraint::TestablePtr getFullCollisionConstraint(
      statespace::dart::MetaSkeletonStateSpacePtr space,
      dart::dynamics::MetaSkeletonPtr skeleton,
      const constraint::dart::CollisionFreePtr& collisionFree) const;

  // TODO(avk): Template this function.
  // Since this is only templated function, might want to make this a
  // convenience function outside of the class.
  ///
  /// Registers a subset of the joints of the skeleton as a new robot.
  ///
  /// \param[in] metaSkeleton The metaskeleton corresponding to the subrobot.
  /// \param[in] name Name of the subrobot.
  virtual std::shared_ptr<Robot> registerSubRobot(
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const std::string& name);

  // TODO(avk): Get type of the robot.
  // TODO(avk): Set root or parent?
  void setRootRobot(Robot* root)
  {
    mRootRobot = root;
  }

  // TODO(avk): Is this function necessary?
  // TODO(avk): Will we want the state from the joint state client?
  statespace::StateSpace::State* getCurrentState() const
  {
    return mStateSpace->getScopedStateFromMetaSkeleton(mMetaSkeleton.get());
  }

  // TODO(avk): Need to introduce a termination condition class.
  trajectory::TrajectoryPtr planToConfiguration(
      const planner::PlannerPtr& planner,
      const statespace::StateSpace::State* goalState,
      const constraint::TestablePtr& testableConstraint) const;

private:
  std::string mName;
  dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  std::shared_ptr<control::TrajectoryExecutor> mTrajectoryExecutor;

  // TODO(avk): This about when this will get deleted.
  Robot* mRootRobot{nullptr};
  std::unordered_map<std::string, std::shared_ptr<Robot>> mChildren;

  // Collision objects.
  dart::collision::CollisionDetectorPtr mCollisionDetector;
  std::shared_ptr<dart::collision::BodyNodeCollisionFilter>
      mSelfCollisionFilter;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_ROBOT_HPP_
