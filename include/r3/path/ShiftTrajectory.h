#ifndef R3_PATH_SHIFTTRAJECTORY_H_
#define R3_PATH_SHIFTTRAJECTORY_H_
#include <r3/path/Spline.h>

namespace r3 {
namespace path {

class ShiftTrajectory : public virtual Trajectory {
public:
  ShiftTrajectory(const ConstTrajectoryPtr& _traj, Scalar _offset);

  Index getNumOutputs() const override;

  Index getNumDerivatives() const override;

  Scalar getDuration() const override;

  Eigen::VectorXd evaluate(Scalar _t, Index _derivative) const override;

private:
  ConstTrajectoryPtr mTrajectory;
  Scalar mOffset;
};

} // namespace path
} // namespace r3

#endif // ifndef R3_PATH_SHIFTTRAJECTORY_H_
