#ifndef AIKIDO_PLANNER_VECTORFIELDPLANNER_HPP_
#define AIKIDO_PLANNER_VECTORFIELDPLANNER_HPP_
#include <Eigen/Core>
#include <boost/function.hpp>
#include <dart/dynamics/SmartPointer.h>
#include <aikido/path/Spline.hpp>
#include <aikido/planner/VectorFieldPlannerExceptions.h>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/statespace/Interpolator.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/constraint/Testable.hpp>

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
  //std::shared_ptr<aikido::statespace::StateSpace> const &stateSpace,
  aikido::statespace::StateSpace::State const *state,
  double t, Eigen::VectorXd *qd)>;

aikido::trajectory::InterpolatedPtr planVF(
  const std::shared_ptr<aikido::statespace::StateSpace>& stateSpace,
  const aikido::statespace::StateSpace::State *startState,
  const aikido::statespace::StateSpace::State *goalState,
  const std::shared_ptr<aikido::statespace::Interpolator>& interpolator,
  const std::shared_ptr<aikido::constraint::Testable>& constraint,
  aikido::planner::PlanningResult& planningResult,
  //dart::dynamics::MetaSkeletonPtr const &skeleton, 
  double dt,
  VectorFieldCallback const &vector_field_cb
  //StatusCallback const &status_cb
  );

} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_VECTORFIELDPLANNER_H_
