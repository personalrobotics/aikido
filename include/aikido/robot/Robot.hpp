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

/// Robot interface for defining basic behaviors of a robot.
/// A concrete implementation of robot should support a collection
/// of planning methods.
class Robot
{
public:
  virtual ~Robot() = default;

  /// Returns a timed trajectory that can be executed by the robot
  /// \param[in] path Geometric path to execute
  /// \param[in] Must be satisfied after postprocessing. Typically
  /// collision constraint is passed.
  virtual trajectory::TrajectoryPtr postprocessPath(
      const trajectory::TrajectoryPtr& path,
      const constraint::TestablePtr& constraint)
      = 0;

  /// Executes a trajectory
  /// \param[in] trajectory Timed trajectory to execute
  virtual void executeTrajectory(
      const trajectory::TrajectoryPtr& trajectory)
      = 0;

  // Returns a named configuration
  /// \param[in] name Name of the configuration
  /// \return COnfiguration
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

  /// \return MetaSkeleton of this robot.
  virtual dart::dynamics::MetaSkeletonPtr getMetaSkeleton() = 0;

  /// \return MetaSkeletonStateSpace of this robot.
  virtual aikido::statespace::dart::MetaSkeletonStateSpacePtr getStateSpace()
      = 0;

  /// Sets the root of this robot.
  /// \param[in] robot Parent robot
  virtual void setRoot(Robot* robot) = 0;

  /// Simulates up to the provided timepoint.
  /// Assumes that parent robot is locked.
  /// \param[in] timepoint Time to simulate to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint) = 0;

  /// Returns self-collision constraint.
  virtual aikido::constraint::CollisionFreePtr getSelfCollisionConstraint() = 0;

  /// Combines provided collision constraint with relf collision
  /// constraint.
  /// \param[in] space Space in which this collision constraint operates.
  /// \param[in] collisionFree Collision constraint.
  /// \return Combined constraint.
  virtual aikido::constraint::TestablePtr getFullCollisionConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr& space,
      const constraint::CollisionFreePtr& collisionFree)
      = 0;
};

} // namespace robot
} // namespace aikido

#endif
