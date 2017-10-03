namespace aikido {
namespace path {

template <Trajectory::Index _NumCoefficients>
SplineTrajectory<_NumCoefficients>::SplineTrajectory(const Spline& _spline)
  : mSpline(_spline)
{
}

template <Trajectory::Index _NumCoefficients>
SplineTrajectory<_NumCoefficients>::SplineTrajectory(Spline&& _spline)
  : mSpline(_spline)
{
}

template <Trajectory::Index _NumCoefficients>
auto SplineTrajectory<_NumCoefficients>::getSpline() -> Spline&
{
  return mSpline;
}

template <Trajectory::Index _NumCoefficients>
auto SplineTrajectory<_NumCoefficients>::getSpline() const -> const Spline&
{
  return mSpline;
}

template <Trajectory::Index _NumCoefficients>
auto SplineTrajectory<_NumCoefficients>::getNumOutputs() const -> Index
{
  return mSpline.getNumOutputs();
}

template <Trajectory::Index _NumCoefficients>
auto SplineTrajectory<_NumCoefficients>::getNumDerivatives() const -> Index
{
  return mSpline.getNumDerivatives();
}

template <Trajectory::Index _NumCoefficients>
auto SplineTrajectory<_NumCoefficients>::getDuration() const -> Scalar
{
  return mSpline.getDuration();
}

template <Trajectory::Index _NumCoefficients>
auto SplineTrajectory<_NumCoefficients>::evaluate(
    Scalar _t, Index _derivative) const -> Vector
{
  return mSpline.evaluate(_t, _derivative);
}

} // namespace path
} // namespace aikido
