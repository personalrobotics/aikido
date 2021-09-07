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
class Robot
{
public:
  virtual ~Robot() = default;

  /// Executes a trajectory
  /// \param[in] trajectory Timed trajectory to execute
  virtual std::future<void> executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory) const = 0;

  /// Returns a named configuration
  /// \param[in] name Name of the configuration
  /// \return Configuration
  virtual boost::optional<Eigen::VectorXd> getNamedConfiguration(
      const std::string& name) const = 0;

  /// Sets the list of named configurations
  /// \param[in] namedConfigurations Map of name, configuration
  virtual void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations)
      = 0;

  /// \return Name of this Robot
  virtual std::string getName() const = 0;

  /// \return [const] MetaSkeleton of this robot.
  virtual dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const = 0;

  /// \return MetaSkeleton of this robot.
  dart::dynamics::MetaSkeletonPtr getMetaSkeleton();

  /// \return [const] MetaSkeletonStateSpace of this robot.
  virtual aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr
  getStateSpace() const = 0;

  /// \return MetaSkeletonStateSpace of this robot.
  aikido::statespace::dart::MetaSkeletonStateSpacePtr getStateSpace();

  /// Sets the root of this robot.
  /// \param[in] robot Parent robot
  virtual void setRoot(Robot* robot) = 0;

  /// Simulates up to the provided timepoint.
  /// \note Assumes that the robot's skeleton is locked.
  /// \param[in] timepoint Time to simulate to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint) = 0;

  /// Returns self-collision constraint.
  /// \param[in] space Space in which this collision constraint operates.
  /// \param[in] metaSkeleton Metaskeleton for the statespace.
  virtual constraint::dart::CollisionFreePtr getSelfCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton) const = 0;

  /// TODO: Consider changing this to return a CollisionFreePtr.
  /// Combines provided collision constraint with self collision
  /// constraint.
  /// \param[in] space Space in which this collision constraint operates.
  /// \param[in] metaSkeleton Metaskeleton for the statespace.
  /// \param[in] collisionFree Collision constraint.
  /// \return Combined constraint.
  virtual constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::ConstMetaSkeletonStateSpacePtr& space,
      const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
      const constraint::dart::CollisionFreePtr& collisionFree) const = 0;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_ROBOT_HPP_
