#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_

#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/Planner.hpp>
#include <aikido/planner/vectorfield/VectorField.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Generate a trajectory following the vector field along given time.
///
/// \param[in] vectorField Vector field to follow.
/// \param[in] startState Start state of the planning.
/// \param[in] constraint Constraint to be satisfied.
/// \param[in] timelimit Timelimit for integration calculation.
/// \param[in] initialStepSize Initial step size of integator in following
/// vector field.
/// \param[in] checkConstraintResolution Resolution used in checking
/// constraint satisfaction in generated trajectory.
/// \param[out] result information about success or failure.
/// \return A trajectory following the vector field.
std::unique_ptr<aikido::trajectory::Spline> followVectorField(
    const aikido::planner::vectorfield::VectorField& vectorField,
    const aikido::statespace::StateSpace::State& startState,
    const aikido::constraint::Testable& constraint,
    std::chrono::duration<double> timelimit,
    double initialStepSize,
    double checkConstraintResolution,
    planner::Planner::Result* result);

/// Plan to a trajectory that moves the end-effector by a given direction and
/// distance.
///
/// \param[in] stateSpace MetaSkeleton state space.
/// \param[in] metaskeleton MetaSkeleton to plan with
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
/// \param[in] initialStepSize Initial step size.
/// \param[in] jointLimitTolerance If less then this distance to joint
/// limit, velocity is bounded in that direction to 0.
/// \param[in] constraintCheckResolution Resolution used in constraint checking.
/// \param[in] timelimit timeout in seconds.
/// \param[out] result information about success or failure.
/// \return Trajectory or \c nullptr if planning failed.
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const statespace::dart::MetaSkeletonStateSpace::State& startState,
    const aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr& stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaskeleton,
    const ::dart::dynamics::ConstBodyNodePtr& bn,
    const aikido::constraint::ConstTestablePtr& constraint,
    const Eigen::Vector3d& direction,
    double minDistance,
    double maxDistance,
    double positionTolerance,
    double angularTolerance,
    double initialStepSize,
    double jointLimitTolerance,
    double constraintCheckResolution,
    std::chrono::duration<double> timelimit,
    planner::Planner::Result* result = nullptr);

/// Plan to an end-effector pose by following a geodesic loss function
/// in SE(3) via an optimized Jacobian.
///
/// \param[in] stateSpace MetaSkeleton state space.
/// \param[in] metaskeleton MetaSkeleton to plan with
/// \param[in] bn Body node of the end-effector.
/// \param[in] constraint Trajectory-wide constraint that must be satisfied.
/// \param[in] goalPose Desired end-effector pose.
/// \param[in] poseErrorTolerance How a planned trajectory is allowed to
/// deviated from a straight line segment defined by the direction and the
/// distance.
/// \param[in] conversionRatioInGeodesicDistance Conversion ratio from radius to
/// meter in computing geodesic distance.
/// \param[in] initialStepSize Initial step size.
/// \param[in] jointLimitTolerance If less then this distance to joint
/// limit, velocity is bounded in that direction to 0.
/// \param[in] constraintCheckResolution Resolution used in constraint checking.
/// \param[in] timelimit Timeout in seconds.
/// \param[out] result information about success or failure.
/// \return Trajectory or \c nullptr if planning failed.
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorPose(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    ::dart::dynamics::MetaSkeletonPtr metaskeleton,
    const ::dart::dynamics::BodyNodePtr& bn,
    const aikido::constraint::TestablePtr& constraint,
    const Eigen::Isometry3d& goalPose,
    double poseErrorTolerance,
    double conversionRatioInGeodesicDistance,
    double initialStepSize,
    double jointLimitTolerance,
    double constraintCheckResolution,
    std::chrono::duration<double> timelimit,
    planner::Planner::Result* result = nullptr);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
