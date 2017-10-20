#ifndef AIKIDO_PLANNER_VECTOR_FIELD_PLANNER_HPP_
#define AIKIDO_PLANNER_VECTOR_FIELD_PLANNER_HPP_

#include <boost/function.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

struct VectorFieldPlannerStatus
{
  enum Enum
  {
    TERMINATE,
    CACHE_AND_TERMINATE,
    CACHE_AND_CONTINUE,
    CONTINUE
  };
};

/// Callback function of joint velocity calculated by a vector field and a
/// MetaSkeleton.
/// \param stateSpace MetaSkeleton state space
/// \param t planned time of a given duration
/// \param qd joint velocity calculated by a vector field and meta skeleton
/// \return trajectory or \c nullptr if planning failed
using VectorFieldCallback = std::function<bool(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t,
    Eigen::VectorXd* qd)>;

/// Callback function of status of planning
/// \param stateSpace MetaSkeleton state space
/// \param t planned time of a given duration
/// \return trajectory or \c nullptr if planning failed
using VectorFieldStatusCallback = std::function<VectorFieldPlannerStatus::Enum(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t)>;

/// Plan to a trajectory by a given vector field.
/// \param stateSpace state space
/// \param constraint trajectory-wide constraint that must be satisfied
/// \param timestep how long an evaluation step is
/// \param vectorField callback of vector field calculation
/// \param statusCb callback of planning status
/// \return trajectory or \c nullptr if planning failed
std::unique_ptr<aikido::trajectory::Spline> planPathByVectorField(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const aikido::constraint::TestablePtr constraint,
    double timestep,
    const VectorFieldCallback& vectorFieldCb,
    const VectorFieldStatusCallback& statusCb);

/// Plan to a trajectory that moves the end-effector by a given direction and
/// distance.
/// \param stateSpace MetaSkeleton state space
/// \param bn body node of the end-effector
/// \param constraint trajectory-wide constraint that must be satisfied
/// \param direction direction of moving the end-effector
/// \param distance  distance of moving the end-effector
/// \param positionTolerance how a planned trajectory is allowed to deviated
/// from
/// a straight line segment defined by the direction and the distance.
/// \param angularTolerance how a planned trajectory is allowed to deviate from
/// a given direction
/// \param duration total time of executing a planned trajectory
/// \param timestep how often velocity should be updated from a vector field
/// \return trajectory or \c nullptr if planning failed
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    dart::dynamics::BodyNodePtr bn,
    const aikido::constraint::TestablePtr constraint,
    const Eigen::Vector3d& direction,
    double distance,
    double positionTolerance = 0.005,
    double angularTolerance = 0.2,
    double duration = 2.0,
    double timestep = 0.01);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTOR_FIELD_PLANNER_HPP_
