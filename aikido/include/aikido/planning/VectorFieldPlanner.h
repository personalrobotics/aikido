#ifndef MUUL_PROJECTION_VECTORFIELDPLANNER_H_
#define MUUL_PROJECTION_VECTORFIELDPLANNER_H_
#include <Eigen/Core>
#include <boost/function.hpp>
#include <dart/dynamics/SmartPointer.h>
#include <aikido/path/Spline.hpp>
#include <muul/projection/VectorFieldPlannerExceptions.h>

namespace muul {
namespace projection {

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

} // namespace projection
} // namespace muul

#endif // ifndef MUUL_PROJECTION_VECTORFIELDPLANNER_H_
