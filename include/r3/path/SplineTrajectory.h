#ifndef R3_PATH_SPLINETRAJECTORY_H_
#define R3_PATH_SPLINETRAJECTORY_H_
#include <r3/path/Trajectory.h>

namespace r3 {
namespace path {

template <class _Spline>
class SplineTrajectory : public virtual Trajectory {
public:
  using Spline = _Spline;

  SplineTrajectory() = default;
  explicit SplineTrajectory(const Spline& _spline);
  explicit SplineTrajectory(Spline&& _spline);
  virtual ~SplineTrajectory() = default;

  // Default copy and move semantics.
  SplineTrajectory(SplineTrajectory&& _other) = default;
  SplineTrajectory(const SplineTrajectory& _other) = default;
  SplineTrajectory& operator =(SplineTrajectory&& _other) = default;
  SplineTrajectory& operator =(const SplineTrajectory& _other) = default;

  const Spline &getSpline() const;

  Index getNumOutputs() const override;

  Index getNumDerivatives() const override;

  Scalar getDuration() const override;

  Eigen::VectorXd evaluate(Scalar _t, Index _derivative) const override;

private:
  Spline mSpline;
};

} // namespace path
} // namespace r3

#include "detail/SplineTrajectory-impl.h"

#endif // ifndef R3_PATH_SPLINETRAJECTORY_H_
