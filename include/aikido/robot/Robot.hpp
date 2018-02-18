#ifndef AIKIDO_ROBOT_ROBOT_HPP_
#define AIKIDO_ROBOT_ROBOT_HPP_

#include <chrono>
#include <string>
#include <unordered_map>
#include <Eigen/Core>
#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/CollisionFree.hpp"
#include "aikido/constraint/TSR.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"
#include "aikido/planner/parabolic/ParabolicTimer.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Robot)

/// A base class for robots.
/// A robot has single metaSkeleton.
/// All planning calls made on this robot should be made on a subset of
/// this robot's metaSkeleton.
class Robot
{
public:
  virtual ~Robot() = default;

  /// Returns a timed trajectory that can be executed by the robot
  /// \param path Geometric path to execute
  virtual trajectory::TrajectoryPtr postprocessPath(
      const trajectory::TrajectoryPtr& path)
      = 0;

  /// Executes a trajectory
  /// \param trajectory Timed trajectory to execute
  virtual void executeTrajectory(const trajectory::TrajectoryPtr& trajectory)
      = 0;

  /// Postprocesses and executes a path
  /// \param timelimit Timelimit for postprocessing.
  virtual void executePath(const trajectory::TrajectoryPtr& path) = 0;

  // Returns a named configuration
  virtual boost::optional<Eigen::VectorXd> getNamedConfiguration(
      const std::string& name) const = 0;

  /// Sets the list of named configurations
  /// \param namedConfigurations Map of name, configuration
  virtual void setNamedConfigurations(
      std::unordered_map<std::string, const Eigen::VectorXd>
          namedConfigurations)
      = 0;

  /// \return Name of this Robot
  virtual std::string getName() const = 0;

  /// Returns the MetaSkeleton of this robot.
  virtual dart::dynamics::MetaSkeletonPtr getMetaSkeleton() = 0;

  /// Sets the root of this robot.
  virtual void setRoot(Robot* robot) = 0;

  /// Simulates up to the provided timepoint.
  /// Assumes that parent robot is locked.
  /// \param timepoint Time to simulate to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint) = 0;

  virtual aikido::constraint::CollisionFreePtr getSelfCollisionConstraint() = 0;

  virtual aikido::constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const constraint::CollisionFreePtr& collisionFree)
      = 0;
};

} // namespace robot
} // namespace aikido

#endif
