#ifndef AIKIDO_ROBOT_UTIL_HPP_
#define AIKIDO_ROBOT_UTIL_HPP_

#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/constraint/CollisionFree.hpp"
#include "aikido/constraint/TSR.hpp"
#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/RNG.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace robot {
namespace util {

// TODO: These are temporary methods until we have Planner API.
  trajectory::InterpolatedPtr planToConfiguration(
                    const MetaSkeletonStateSpacePtr &space,
                    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
                    const statespace::StateSpace::State* startState,
                    const statespace::StateSpace::State* goalState,
                    double timelimit,
                    const TestablePtr &collisionTestable, RNG *rng,
                    double collisionResolution);


  trajectory::InterpolatedPtr planToConfigurations(
                    const MetaSkeletonStateSpacePtr &space,
                    const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
                    const statespace::StateSpace::State* startState,
                    const std::vector<tatespace::StateSpace::State*> goalStates,
                    double timelimit,
                    const TestablePtr &collisionTestable, RNG *rng,
                    double collisionResolution);


  trajectory::InterpolatedPtr planToTSR(
                          const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
                          const MetaSkeletonStateSpacePtr &space,
                          const BodyNodePtr &bn, const TSRPtr &tsr,
                          int maxNumTrials, double timelimit,
                          const TestablePtr &collisionTestable, RNG *rng,
                          double collisionResolution);


  /// Returns a Trajectory that moves the configuration of the metakeleton such
  /// that the specified bodynode is set to a sample in a goal TSR and
  /// the trajectory is constrained to a constraint TSR
  /// \param space The StateSpace for the metaskeleton
  /// \param body Bodynode whose frame is meant for TSR
  /// \param goalTsr The goal TSR to move to
  /// \param constraintTsr The constraint TSR for the trajectory
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  trajectory::TrajectoryPtr planToTSRwithTrajectoryConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &bodyNode,
      const constraint::TSRPtr &goalTsr,
      const constraint::TSRPtr &constraintTsr,
      const constraint::CollisionFreePtr &collisionFree);


  /// Plan to a desired end-effector offset with fixed orientation.
  /// \param space StateSpace for the metaskeleton
  /// \param body Bodynode for the end effector
  /// \param metaSkeleton Metaskeleton to plan with
  /// \param direction Direction unit vector in the world frame
  /// \param distance Distance distance to move, in meters
  /// \param collisionFree CollisionFree constraint to check. Self-collision
  /// is checked by default.
  /// \return Output trajectory
  trajectory::TrajectoryPtr planToEndEffectorOffset(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &body,
      const Eigen::Vector3d &direction,
      const constraint::CollisionFreePtr &collisionFree,
      double distance,
      double linearVelocity);


  /// Plan to a desired end-effector offset along the z axis of the
  /// end-effector. Tries VF and then CRRT.
  /// \return Output trajectory
  trajectory::TrajectoryPtr planEndEffectorStraight(
      statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &body,
      const constraint::CollisionFreePtr &collisionFree,
      double distance,
      double linearVelocity);

}
}
}

#endif


