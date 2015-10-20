#ifndef R3_PATH_TRAJECTORY_H_
#define R3_PATH_TRAJECTORY_H_
#include <memory>

namespace r3 {
namespace path {

class Trajectory
{
public:
  using Scalar = double;
  using Index = ptrdiff_t;

  virtual ~Trajectory() = default;

  virtual Index getNumOutputs() const = 0;

  virtual Index getNumDerivatives() const = 0;

  virtual Scalar getDuration() const = 0;

  virtual Eigen::VectorXd evaluate(Scalar _t, Index _derivative) const = 0;
};

using TrajectoryPtr = std::shared_ptr<Trajectory>;
using ConstTrajectoryPtr = std::shared_ptr<const Trajectory>;

} // namespace path
} // namespace r3

#endif // ifndef R3_PATH_TRAJECTORY_H_
