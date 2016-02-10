#ifndef AIKIDO_PATH_BLENDTRAJECTORY_H_
#define AIKIDO_PATH_BLENDTRAJECTORY_H_
#include "Spline.hpp"

namespace aikido {
namespace path {

class BlendTrajectory : public virtual Trajectory {
public:
  BlendTrajectory(
    const ConstTrajectoryPtr& _traj1,
    const ConstTrajectoryPtr& _traj2,
    double _timeStart,
    double _TimeEnd);

  Index getNumOutputs() const override;

  Index getNumDerivatives() const override;

  Scalar getDuration() const override;

  double getBlendDuration();

  Eigen::VectorXd evaluate(Scalar _t, Index _derivative) const override;

private:
  ConstTrajectoryPtr mTrajectory1;
  ConstTrajectoryPtr mTrajectory2;
  double mTimeStart;
  double mTimeEnd;
};

} // namespace path
} // namespace aikido

#endif // AIKIDO_PATH_BLENDTRAJECTORY_H_
