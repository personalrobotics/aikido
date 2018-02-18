#ifndef AIKIDO_ROBOT_UTIL_HPP_
#define AIKIDO_ROBOT_UTIL_HPP_

#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/common/ExecutorThread.hpp"
#include "aikido/common/RNG.hpp"
#include "aikido/constraint/CollisionFree.hpp"
#include "aikido/constraint/TSR.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/control/TrajectoryExecutor.hpp"
#include "aikido/io/yaml.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace robot {

// TODO: These are planning methods used in Robot classes. These will be
// removed once we have a Planner API.
namespace util {

static const double kTimelimit = 3.0;

/// Plan the robot to a specific configuration.
/// Restores the robot to its initial configuration after planning.
/// \param[in] space The StateSpace for the metaskeleton
/// \param[in] metaSkeleton MetaSkeleton to plan with.
/// \param[in] goalState Goal state
/// \param[in] collisionTestable Testable constraint to check for collision.
/// \param[in] rng Random number generator
/// \param[in] timelimit Max time to spend per planning to each IK
trajectory::InterpolatedPtr planToConfiguration(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const statespace::StateSpace::State* goalState,
    const constraint::TestablePtr& collisionTestable,
    common::RNG* rng,
    double timlimit = kTimelimit);

/// Plan the robot to a set of configurations.
/// Restores the robot to its initial configuration after planning.
/// \param[in] space The StateSpace for the metaskeleton
/// \param[in] metaSkeleton MetaSkeleton to plan with.
/// \param[in] goalStateis Goal states
/// \param[in] collisionTestable Testable constraint to check for collision.
/// \param[in] rng Random number generator
/// \param[in] timelimit Max time to spend per planning to each IK
trajectory::InterpolatedPtr planToConfigurations(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const std::vector<statespace::StateSpace::State*> goalStates,
    const constraint::TestablePtr& collisionTestable,
    common::RNG* rng,
    double timlimit = kTimelimit);

/// Plan the configuration of the metakeleton such that
/// the specified bodynode is set to a sample in TSR
/// \param[in] space The StateSpace for the metaskeleton.
/// \param[in] metaSkeleton MetaSkeleton to plan with.
/// \param[in] bodyNode Bodynode whose frame for which TSR is constructed.
/// \param[in] collisionTestable Testable constraint to check for collision.
/// \param[in] rng Random number generator
/// \param[in] timelimit Max time (seconds) to spend per planning to each IK
/// \return Trajectory to a sample in TSR, or nullptr if planning fails.
trajectory::InterpolatedPtr planToTSR(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const constraint::TSRPtr& tsr,
    const constraint::TestablePtr& collisionTestable,
    common::RNG* RNG,
    double timelimit = kTimelimit);

/// Returns a Trajectory that moves the configuration of the metakeleton such
/// that the specified bodynode is set to a sample in a goal TSR and
/// the trajectory is constrained to a constraint TSR
/// \param[in] space The StateSpace for the metaskeleton
/// \param[in] metaSkeleton MetaSkeleton to plan with.
/// \param[in] body Bodynode whose frame is meant for TSR
/// \param[in] goalTsr The goal TSR to move to
/// \param[in] constraintTsr The constraint TSR for the trajectory
/// \param[in] timelimit Timelimit for planning
/// \param[in] projectionMaxIteration Parameter for maximum iteration of
/// constraint projection
/// \param[in] projectionTolerance Parameter for projection tolerance
/// \param[in] maxExtensionDistance Parameter for CRRTConnect
/// \param[in] maxDistanceBtwProjections Parameter for CRRTConnect
/// \param[in] minStepsize Parameter for CRRTConnect
/// \param[in] minTreeConnectionDistance Parameter for CRRTConnect
/// \return Trajectory to a sample in TSR, or nullptr if planning fails.
trajectory::InterpolatedPtr planToTSRwithTrajectoryConstraint(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& bodyNode,
    const constraint::TSRPtr& goalTsr,
    const constraint::TSRPtr& constraintTsr,
    const constraint::TestablePtr& collisionTestable,
    double timelimit = kTimelimit,
    int projectionMaxIteration = 20,
    double projectionTolerance = 1e-4,
    double maxExtensionDistance = std::numeric_limits<double>::infinity(),
    double maxDistanceBtwProjections = 0.1,
    double minStepsize = 0.05,
    double minTreeConnectionDistance = 0.1);

/// Plan to a desired end-effector offset with fixed orientation.
/// \param[in] space StateSpace for the metaskeleton
/// \param[in] metaSkeleton Metaskeleton to plan with
/// \param[in] body Bodynode for the end effector
/// \param[in] direction Direction unit vector in the world frame
/// \param[in] collisionTestable Collision constraint to check. Self-collision
/// is checked by default.
/// \param[in] distance Distance distance to move, in meters
/// \param[in] timelimit Timelimit for planning
/// \param[in] disanceTolearance Distance tolerance
/// \param[in] positionTolerance Position tolerance
/// \param[in] angularTolerance Angular tolerance
/// \param[in] initialStepSize Initial step size.
/// \param[in] jointLimitTolerance If less then this distance to joint.
/// \param[in] constraintCheckResolution Resolution used in constraint checking.
/// \return Output trajectory
trajectory::SplinePtr planToEndEffectorOffset(
    const statespace::dart::MetaSkeletonStateSpacePtr& space,
    const dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const dart::dynamics::BodyNodePtr& body,
    const Eigen::Vector3d& direction,
    const constraint::TestablePtr& collisionTestable,
    double distance,
    double timelimit = kTimelimit,
    double distanceTolerance = 0.1,
    double positionTolerance = 1e-3,
    double angularTolerance = 1e-3,
    double initialStepSize = 3.0,            // TODO: find the right default
    double jointLimitTolerance = 3.0,        // TODO: find the right default
    double constraintCheckResolution = 3.0); // TODO: find the right default

std::unordered_map<std::string, const Eigen::VectorXd>
parseYAMLToNamedConfigurations(const YAML::Node& node);
}
}
}

#endif
