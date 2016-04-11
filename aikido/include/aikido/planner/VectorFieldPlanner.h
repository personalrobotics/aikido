#ifndef AIKIDO_PLANNER_VECTORFIELDPLANNER_H_
#define AIKIDO_PLANNER_VECTORFIELDPLANNER_H_
#include <Eigen/Core>
#include <boost/function.hpp>
#include <dart/dynamics/SmartPointer.h>
#include <aikido/path/Spline.hpp>
#include <aikido/planner/VectorFieldPlannerExceptions.h>

namespace aikido {
namespace planner {

struct Status {
  enum Enum {
    TERMINATE,
    CACHE_AND_TERMINATE,
    CACHE_AND_CONTINUE,
    CONTINUE
  };
};

using StatusCallback = boost::function<Status::Enum (
  dart::dynamics::MetaSkeletonPtr const &skeleton, double t)>;

using VectorFieldCallback = boost::function<bool (
  dart::dynamics::MetaSkeletonPtr const &skeleton,
  double t, Eigen::VectorXd *qd)>;

aikido::path::TrajectoryPtr Plan(
  dart::dynamics::MetaSkeletonPtr const &skeleton, double dt,
  VectorFieldCallback const &vector_field_cb,
  StatusCallback const &status_cb);

} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_VECTORFIELDPLANNER_H_
