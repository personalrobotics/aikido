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

using VectorFieldCallback = std::function<bool(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t,
    Eigen::VectorXd* qd)>;

using VectorFieldStatusCallback = std::function<VectorFieldPlannerStatus::Enum(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double duration)>;

std::unique_ptr<aikido::trajectory::Spline> planPathByVectorField(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    const aikido::constraint::TestablePtr constraint,
    double timestep,
    const VectorFieldCallback& vectorFiledCb,
    const VectorFieldStatusCallback& statusCb);

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
