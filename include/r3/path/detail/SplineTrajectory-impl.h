namespace r3 {
namespace path {

template <class _Spline>
SplineTrajectory<_Spline>::SplineTrajectory(const Spline& _spline)
  : mSpline(_spline)
{
}

template <class _Spline>
SplineTrajectory<_Spline>::SplineTrajectory(Spline&& _spline)
  : mSpline(_spline)
{
}

template <class _Spline>
auto SplineTrajectory<_Spline>::getSpline() const -> const Spline &
{
  return mSpline;
}

template <class _Spline>
auto SplineTrajectory<_Spline>::getNumOutputs() const -> Index
{
  return mSpline.getNumOutputs();
}

template <class _Spline>
auto SplineTrajectory<_Spline>::getNumDerivatives() const -> Index
{
  return mSpline.getNumDerivatives();
}

template <class _Spline>
auto SplineTrajectory<_Spline>::getDuration() const -> Scalar
{
  return mSpline.getDuration();
}

template <class _Spline>
Eigen::VectorXd SplineTrajectory<_Spline>
  ::evaluate(Scalar _t, Index _derivative) const
{
  return mSpline.evaluate(_t, _derivative);
}

} // namespace path
} // namespace r3
