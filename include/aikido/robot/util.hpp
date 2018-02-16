#ifndef AIKIDO_ROBOT_UTIL_HPP_
#define AIKIDO_ROBOT_UTIL_HPP_

#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/constraint/CollisionFree.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/constraint/TSR.hpp"
#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/RNG.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace robot {

// TODO: These are planning methods used in Robot classes. These will be
// removed once we have a Planner API.
namespace util {

  static const double kTimelimit = 3.0;

  trajectory::InterpolatedPtr planToConfiguration(
   const statespace::dart::MetaSkeletonStateSpacePtr &space,
   const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
   const statespace::StateSpace::State* goalState,
   const constraint::TestablePtr &collisionTestable, common::RNG *rng,
   double timlimit = kTimelimit);


  trajectory::InterpolatedPtr planToConfigurations(
  const statespace::dart::MetaSkeletonStateSpacePtr &space,
  const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
  const std::vector<statespace::StateSpace::State*> goalStates,
  const constraint::TestablePtr &collisionTestable, common::RNG *rng,
  double timlimit = kTimelimit);

  trajectory::InterpolatedPtr planToTSR(
 const statespace::dart::MetaSkeletonStateSpacePtr &space,
 const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
 const dart::dynamics::BodyNodePtr &bn, const constraint::TSRPtr &tsr,
 const constraint::TestablePtr &collisionTestable, common::RNG *RNG,
  double timelimit = kTimelimit
 );



  /// Returns a Trajectory that moves the configuration of the metakeleton such
  /// that the specified bodynode is set to a sample in a goal TSR and
  /// the trajectory is constrained to a constraint TSR
  /// \param space The StateSpace for the metaskeleton
  /// \param body Bodynode whose frame is meant for TSR
  /// \param goalTsr The goal TSR to move to
  /// \param constraintTsr The constraint TSR for the trajectory
  /// \return Trajectory to a sample in TSR, or nullptr if planning fails.
  trajectory::InterpolatedPtr planToTSRwithTrajectoryConstraint(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &bodyNode,
      const constraint::TSRPtr &goalTsr,
      const constraint::TSRPtr &constraintTsr,
      const constraint::TestablePtr &collisionTestable,
      double timelimit = kTimelimit,
    int projectionMaxIteration = 3,
    double projectionTolerance = 3.0,
    double maxExtensionDistance = 3.0,
    double maxDistanceBtwProjections = 3.0,
    double minStepsize = 3.0,
    double minTreeConnectionDistance = 3.0);

  /// Plan to a desired end-effector offset with fixed orientation.
  /// \param space StateSpace for the metaskeleton
  /// \param body Bodynode for the end effector
  /// \param metaSkeleton Metaskeleton to plan with
  /// \param direction Direction unit vector in the world frame
  /// \param distance Distance distance to move, in meters
  /// \param collisionFree CollisionFree constraint to check. Self-collision
  /// is checked by default.
  /// \return Output trajectory
  trajectory::SplinePtr planToEndEffectorOffset(
      const statespace::dart::MetaSkeletonStateSpacePtr &space,
      const dart::dynamics::MetaSkeletonPtr &metaSkeleton,
      const dart::dynamics::BodyNodePtr &body,
      const Eigen::Vector3d &direction,
      const constraint::TestablePtr &collisionTestable,
      double distance,
      double linearVelocity,
      double timelimit = kTimelimit,
    double minDistance = 3.0,
    double maxDistance = 3.0,
    double positionTolerance = 3.0,
    double angularTolerance = 3.0,
    double initialStepSize = 3.0,
    double jointLimitTolerance = 3.0,
    double constraintCheckResolution = 3.0);

}
}
}

#endif


