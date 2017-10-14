#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEHANDSTRAIGHTVECTORFIELD_H_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEHANDSTRAIGHTVECTORFIELD_H_
#include <Eigen/Geometry>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

class MoveHandStraightVectorField {
public:
  MoveHandStraightVectorField(
    dart::dynamics::BodyNodePtr bn,
    Eigen::Vector3d const &linear_velocity,
    double min_duration,
    double max_duration,
    double stepsize,
    double linear_gain = 10.,
    double linear_tolerance = 0.01,
    double rotation_gain = 10.,
    double rotation_tolerance = 0.01,
    double optimization_tolerance = 1e-4,
    double padding = 1e-5
  );

  bool operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t, Eigen::VectorXd *qd);

  VectorFieldPlannerStatus::Enum operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t);

private:
  dart::dynamics::BodyNode *bodynode_;
  Eigen::VectorXd velocity_;
  double min_duration_;
  double max_duration_;
  double timsetep_;
  double linear_gain_;
  double linear_tolerance_;
  double rotation_gain_;
  double rotation_tolerance_;
  double optimization_tolerance_;
  double padding_;
  Eigen::Isometry3d start_pose_;
  Eigen::Isometry3d target_pose_;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEHANDSTRAIGHTVECTORFIELD_H_
