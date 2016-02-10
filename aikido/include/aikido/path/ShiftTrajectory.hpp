#ifndef AIKIDO_PATH_SHIFTTRAJECTORY_H_
#define AIKIDO_PATH_SHIFTTRAJECTORY_H_
#include "Spline.hpp"

namespace aikido {
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
} // namespace aikido

#endif // AIKIDO_PATH_SHIFTTRAJECTORY_H_
