#ifndef AIKIDO_PATH_TWISTTRAJECTORY_H_
#define AIKIDO_PATH_TWISTTRAJECTORY_H_

#include "Trajectory.h"
#include "CartesianTrajectory.h"

namespace aikido {
namespace path {

class SE3TwistTrajectory : public virtual SE3Trajectory
{
public:
  explicit SE3TwistTrajectory(const TrajectoryPtr& _twistTrajectory);

  Index getNumDerivatives() const override;

  Scalar getDuration() const override;

  Output evaluate(Scalar _t) const override;

private:
  TrajectoryPtr mTwistTrajectory;
};

} // namespace path
} // namespace aikido

#endif // AIKIDO_PATH_TWISTTRAJECTORY_H_
