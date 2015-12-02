#ifndef R3_PATH_CARTESIANTRAJECTORY_H_
#define R3_PATH_CARTESIANTRAJECTORY_H_
#include <memory>

namespace r3 {
namespace path {

template <int _Dim>
class CartesianTrajectory
{
public:
  using Index = int;
  using Scalar = double;
  using Output = Eigen::Transform<Scalar, _Dim, Eigen::Isometry>;

  static constexpr int Dimension = _Dim;

  virtual ~CartesianTrajectory() = default;

  virtual Index getNumDerivatives() const = 0;

  virtual Scalar getDuration() const = 0;

  virtual Output evaluate(Scalar _t) const = 0;
};

using SE2Trajectory = CartesianTrajectory<2>;
using SE2TrajectoryPtr = std::shared_ptr<SE2Trajectory>;
using ConstSE2TrajectoryPtr = std::shared_ptr<const SE2Trajectory>;

using SE3Trajectory = CartesianTrajectory<3>;
using SE3TrajectoryPtr = std::shared_ptr<SE3Trajectory>;
using ConstSE3TrajectoryPtr = std::shared_ptr<const SE3Trajectory>;

} // namespace path
} // namespace r3

#endif // ifndef R3_PATH_TRAJECTORY_H_
