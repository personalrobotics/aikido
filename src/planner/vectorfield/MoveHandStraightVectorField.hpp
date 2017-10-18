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
    Eigen::Vector3d const &linearVelocity,
    double minDuration,
    double maxDuration,
    double stepsize,
    double linearGain = 1.,
    double linearTolerance = 0.01,
    double rotationGain = 1.,
    double rotationTolerance = 0.01,
    double optimizationTolerance = 1e-3,
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
  double minDuration_;
  double maxDuration_;
  double timestep_;
  double linearGain_;
  double linearTolerance_;
  double rotationGain_;
  double rotationTolerance_;
  double optimizationTolerance_;
  double padding_;
  Eigen::Isometry3d startPose_;
  Eigen::Isometry3d targetPose_;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEHANDSTRAIGHTVECTORFIELD_H_
