#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_

#include <boost/function.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

enum class VectorFieldPlannerStatus
{
  TERMINATE,
  CACHE_AND_TERMINATE,
  CACHE_AND_CONTINUE,
  CONTINUE
};

/// Callback function of joint velocity calculated by a vector field and a
/// MetaSkeleton.
///
/// \param[in] _stateSpace MetaSkeleton state space
/// \param[in] _t Planned time of a given duration
/// \param[out] _qd Joint velocity calculated by a vector field and meta
/// skeleton
/// \return Whether vectorfield evaluation succeeds
using VectorFieldCallback = std::function<bool(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    double _t,
    Eigen::VectorXd* _qd)>;

/// Callback function of status of planning
///
/// \param[in] _stateSpace MetaSkeleton state space
/// \param[in] _t Planned time of a given duration
/// \return Status of vectorfield planner
using VectorFieldStatusCallback = std::function<VectorFieldPlannerStatus(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    double _t)>;

/// Plan to a trajectory by a given vector field.
///
/// \param[in] _stateSpace MetaSkeleton state space
/// \param[in] _constraint Trajectory-wide constraint that must be satisfied
/// \param[in] _timestep How long an evaluation step is
/// \param[in] _vectorField Callback of vector field calculation
/// \param[in] _statusCb Callback of planning status
/// \return Trajectory or \c nullptr if planning failed
std::unique_ptr<aikido::trajectory::Spline> planPathByVectorField(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    const aikido::constraint::TestablePtr _constraint,
    double _timestep,
    const VectorFieldCallback& _vectorFieldCb,
    const VectorFieldStatusCallback& _statusCb);

/// Plan to a trajectory that moves the end-effector by a given direction and
/// distance.
///
/// \param[in] _stateSpace MetaSkeleton state space
/// \param[in] _bn Body node of the end-effector
/// \param[in] _constraint Trajectory-wide constraint that must be satisfied
/// \param[in] _direction Direction of moving the end-effector
/// \param[in] _distance  Distance of moving the end-effector
/// \param[in] _positionTolerance How a planned trajectory is allowed to
/// deviated
/// from a straight line segment defined by the direction and the distance.
/// \param angularTolerance How a planned trajectory is allowed to deviate from
/// a given direction
/// \param[in] _duration Total time of executing a planned trajectory
/// \param[in] _timestep How often velocity should be updated from a vector
/// field
/// \return Trajectory or \c nullptr if planning failed
std::unique_ptr<aikido::trajectory::Spline> planToEndEffectorOffset(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    dart::dynamics::BodyNodePtr _bn,
    const aikido::constraint::TestablePtr _constraint,
    const Eigen::Vector3d& _direction,
    double _distance,
    double _positionTolerance = 0.005,
    double _angularTolerance = 0.2,
    double _duration = 2.0,
    double _timestep = 0.01);

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNER_HPP_
