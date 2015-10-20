#ifndef R3_PATH_TWISTTRAJECTORY_H_
#define R3_PATH_TWISTTRAJECTORY_H_

#include <r3/path/Trajectory.h>
#include <r3/path/CartesianTrajectory.h>

namespace r3 {
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
} // namespace r3

#endif // ifndef R3_PATH_TWISTTRAJECTORY_H_
