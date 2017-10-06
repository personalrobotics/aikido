#ifndef MUUL_PROJECTION_MOVEHANDSTRAIGHTVECTORFIELD_H_
#define MUUL_PROJECTION_MOVEHANDSTRAIGHTVECTORFIELD_H_
#include <Eigen/Geometry>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

class MoveHandStraightVectorField {
public:
  MoveHandStraightVectorField(
    dart::dynamics::BodyNode *bodynode,
    Eigen::Vector3d const &linear_velocity,
    double min_duration,
    double max_duration,
    double dt,
    double linear_gain = 10.,
    double linear_tolerance = 0.01,
    double rotation_gain = 10.,
    double rotation_tolerance = 0.1
  );

  bool operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    double t, Eigen::VectorXd *qd);

  VectorFieldPlannerStatus::Enum operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
    double t);

private:
  double min_duration_;
  double max_duration_;
  double dt_;
  Eigen::VectorXd velocity_;
  double linear_gain_;
  double linear_tolerance_;
  double rotation_gain_;
  double rotation_tolerance_;
  dart::dynamics::BodyNode *bodynode_;
  Eigen::Isometry3d start_pose_;
  Eigen::Isometry3d target_pose_;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef MUUL_PROJECTION_MOVEHANDSTRAIGHTVECTORFIELD_H_
