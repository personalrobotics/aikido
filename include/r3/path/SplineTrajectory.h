#ifndef R3_PATH_SPLINETRAJECTORY_H_
#define R3_PATH_SPLINETRAJECTORY_H_
#include <r3/path/Spline.h>
#include <r3/path/Trajectory.h>

namespace r3 {
namespace path {


template <Trajectory::Index _NumCoefficients = Eigen::Dynamic>
class SplineTrajectory : public virtual Trajectory {
public:
  using Spline = SplineND<
    Scalar, Index, _NumCoefficients, Eigen::Dynamic, Eigen::Dynamic>;
  using SplineProblem = SplineProblem<
    Scalar, Index, _NumCoefficients, Eigen::Dynamic, Eigen::Dynamic>;

  SplineTrajectory() = default;
  explicit SplineTrajectory(const Spline& _spline);
  explicit SplineTrajectory(Spline&& _spline);
  virtual ~SplineTrajectory() = default;

  // Default copy and move semantics.
  SplineTrajectory(SplineTrajectory&& _other) = default;
  SplineTrajectory(const SplineTrajectory& _other) = default;
  SplineTrajectory& operator =(SplineTrajectory&& _other) = default;
  SplineTrajectory& operator =(const SplineTrajectory& _other) = default;

  Spline& getSpline();

  const Spline& getSpline() const;

  Index getNumOutputs() const override;

  Index getNumDerivatives() const override;

  Scalar getDuration() const override;

  Vector evaluate(Scalar _t, Index _derivative) const override;

private:
  Spline mSpline;
};


using LinearSplineTrajectory = SplineTrajectory<2>;
using LinearSplineTrajectoryPtr
  = boost::shared_ptr<LinearSplineTrajectory>;
using ConstLinearSplineTrajectoryPtr
  = boost::shared_ptr<const LinearSplineTrajectory>;


using CubicSplineTrajectory = SplineTrajectory<4>;
using CubicSplineTrajectoryPtr
  = boost::shared_ptr<CubicSplineTrajectory>;
using ConstCubicSplineTrajectoryPtr
  = boost::shared_ptr<const CubicSplineTrajectory>;


using QuinticSplineTrajectory = SplineTrajectory<6>;
using QuinticSplineTrajectoryPtr
  = boost::shared_ptr<QuinticSplineTrajectory>;
using ConstQuinticSplineTrajectoryPtr
  = boost::shared_ptr<const QuinticSplineTrajectory>;


} // namespace path
} // namespace r3

#include "detail/SplineTrajectory-impl.h"

#endif // ifndef R3_PATH_SPLINETRAJECTORY_H_
