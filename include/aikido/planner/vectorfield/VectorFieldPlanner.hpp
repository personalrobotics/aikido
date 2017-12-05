#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_

#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Plan to a trajectory that moves the end-effector by a given direction and
/// distance.
///
/// \param[in] stateSpace MetaSkeleton state space.
/// \param[in] bn Body node of the end-effector.
/// \param[in] constraint Trajectory-wide constraint that must be satisfied.
/// \param[in] direction Direction of moving the end-effector.
/// \param[in] minDistance  Distance of moving the end-effector.
/// \param[in] maxDistance Max distance of moving the end-effector.
/// \param[in] positionTolerance How a planned trajectory is allowed to
/// deviated from a straight line segment defined by the direction and the
/// distance.
/// \param[in] angularTolerance How a planned trajectory is allowed to deviate
/// from a given direction.
/// \param[in] linearVelocityGain Linear velocity gain in workspace.
/// \param[in] initialStepSize Initial step size.
/// \param[in] jointLimitTolerance If less then this distance to joint
/// limit, velocity is bounded in that direction to 0.
/// \param[in] constraintCheckResolution Resolution used in constraint checking.
/// \param[in] timelimit timeout in seconds.
/// \param[out] planningResult information about success or failure.
/// \return Trajectory or \c nullptr if planning failed.
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Vector3d& direction,
    double minDistance,
    double maxDistance,
    double positionTolerance,
    double angularTolerance,
    double linearVelocityGain,
    double initialStepSize,
    double jointLimitTolerance,
    double constraintCheckResolution,
    std::chrono::duration<double> timelimit,
    planner::PlanningResult& planningResult);

/// Plan to an end effector pose by following a geodesic loss function
/// in SE(3) via an optimized Jacobian.
///
/// \param[in] stateSpace MetaSkeleton state space.
/// \param[in] bn Body node of the end-effector.
/// \param[in] constraint Trajectory-wide constraint that must be satisfied.
/// \param[in] goalPose Desired end-effector pose.
/// \param[in] positionErrorTolerance How a planned trajectory is allowed to
/// deviated from a straight line segment defined by the direction and the
/// distance.
/// \param[in] linearVelocityGain Linear velocity gain in workspace.
/// \param[in] angularVelocityGain Angular velocity gain in workspace.
/// \param[in] initialStepSize Initial step size.
/// \param[in] jointLimitTolerance If less then this distance to joint
/// limit, velocity is bounded in that direction to 0.
/// \param[in] constraintCheckResolution Resolution used in constraint checking.
/// \param[in] timelimit Timeout in seconds.
/// \param[out] planningResult information about success or failure.
/// \return Trajectory or \c nullptr if planning failed.
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorPose(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    const dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Isometry3d& goalPose,
    double poseErrorTolerance,
    double linearVelocityGain,
    double angularVelocityGain,
    double initialStepSize,
    double jointLimitTolerance,
    double constraintCheckResolution,
    std::chrono::duration<double> timelimit,
    planner::PlanningResult& planningResult);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
