#ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEHANDSTRAIGHTVECTORFIELD_H_
#define AIKIDO_PLANNER_VECTORFIELD_MOVEHANDSTRAIGHTVECTORFIELD_H_

#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

class MoveHandStraightVectorField {
public:
  MoveHandStraightVectorField(
    dart::dynamics::BodyNodePtr _bn,
    const Eigen::Vector3d& _linearVelocity,
    double _minDuration,
    double _maxDuration,
    double _stepsize,
    double _linearGain = 1.,
    double _linearTolerance = 0.01,
    double _rotationGain = 1.,
    double _rotationTolerance = 0.01,
    double _optimizationTolerance = 1e-3,
    double _padding = 1e-5
  );

  bool operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    double _t,
    Eigen::VectorXd* _qd);

  VectorFieldPlannerStatus operator()(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace,
    double t);

private:
  dart::dynamics::BodyNodePtr mBodynode;
  Eigen::VectorXd mVelocity;
  Eigen::VectorXd mLinearDirection;
  double mMinDuration;
  double mMaxDuration;
  double mTimestep;
  double mLinearGain;
  double mLinearTolerance;
  double mRotationGain;
  double mRotationTolerance;
  double mOptimizationTolerance;
  double mPadding;
  Eigen::Isometry3d mStartPose;
  Eigen::Isometry3d mTargetPose;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // ifndef AIKIDO_PLANNER_VECTORFIELD_MOVEHANDSTRAIGHTVECTORFIELD_H_
