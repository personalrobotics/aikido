#ifndef AIKIDO_PLANNER_VECTOR_FIELD_PLANNER_HPP_
#define AIKIDO_PLANNER_VECTOR_FIELD_PLANNER_HPP_

#include <dart/dynamics/MetaSkeleton.hpp>
#include <aikido/trajectory/Interpolated.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

struct VectorFieldPlannerStatus {
  enum Enum {
    TERMINATE,
    CACHE_AND_TERMINATE,
    CACHE_AND_CONTINUE,
    CONTINUE
  };
};

using VectorFieldCallback = std::function<bool (
  const dart::dynamics::MetaSkeletonPtr skeleton,
  double t,
  const Eigen::VectorXd& qd)>;

using VectorFieldStatusCallback = 
  std::function<VectorFieldPlannerStatus::Enum (
  const dart::dynamics::MetaSkeletonPtr skeleton, double t)>; 

aikido::trajectory::InterpolatedPtr planPathByVectorField(
  const dart::dynamics::MetaSkeletonPtr skeleton,
  double t,
  const VectorFieldCallback& vectorFiledCb,
  const VectorFieldStatusCallback& statusCb);

aikido::trajectory::InterpolatedPtr planStrightLine(
  const dart::dynamics::MetaSkeletonPtr skeleton,
  const Eigen::VectorXd& startPosition,
  const Eigen::VectorXd& goalPosition); 

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTOR_FIELD_PLANNER_HPP_
