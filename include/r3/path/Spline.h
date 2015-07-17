#ifndef R3_PATH_SPLINE_H_
#define R3_PATH_SPLINE_H_

#include <cstddef>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/StdVector>

namespace r3 {
namespace path {

template <
  class _Scalar = double,
  class _Index = ptrdiff_t,
  _Index _NumCoefficients = Eigen::Dynamic,
  _Index _NumOutputs = Eigen::Dynamic,
  _Index _NumKnots = Eigen::Dynamic>
class SplineProblem
{
public:
  using Scalar = _Scalar;
  using Index = _Index;

  static constexpr Index NumCoefficientsAtCompileTime = _NumCoefficients;
  static constexpr Index NumOutputsAtCompileTime = _NumOutputs;
  static constexpr Index NumKnotsAtCompileTime= _NumKnots;
  static constexpr Index NumSegmentsAtCompileTime
    = (_NumKnots != Eigen::Dynamic)
      ? (NumKnotsAtCompileTime - 1)
      : Eigen::Dynamic;
  static constexpr Index DimensionAtCompileTime
    = (NumSegmentsAtCompileTime != Eigen::Dynamic && _NumCoefficients != Eigen::Dynamic)
      ? (NumSegmentsAtCompileTime * NumCoefficientsAtCompileTime)
      : Eigen::Dynamic;

  using TimeVector = Eigen::Matrix<Scalar, NumKnotsAtCompileTime, 1>;
  using OutputVector = Eigen::Matrix<Scalar, NumOutputsAtCompileTime, 1>;
  using OutputMatrix = Eigen::Matrix<Scalar, NumCoefficientsAtCompileTime, NumOutputsAtCompileTime>;
  using CoefficientVector = Eigen::Matrix<Scalar, NumCoefficientsAtCompileTime, 1>;
  using CoefficientMatrix = Eigen::Matrix<Scalar, NumCoefficientsAtCompileTime, NumCoefficientsAtCompileTime>;
  using ProblemMatrix = Eigen::Matrix<Scalar, DimensionAtCompileTime, DimensionAtCompileTime>;
  using ProblemVector = Eigen::Matrix<Scalar, DimensionAtCompileTime, NumOutputsAtCompileTime>;
  using SolutionMatrix = Eigen::Matrix<Scalar, NumOutputsAtCompileTime, NumCoefficientsAtCompileTime>;

  explicit SplineProblem(const TimeVector& _times);
  SplineProblem(const TimeVector& _times, Index _numCoefficients, Index _numOutputs);

  CoefficientVector createTimeVector(Scalar _t, Index _i) const;
  CoefficientMatrix createTimeMatrix(Scalar _t) const;
  CoefficientMatrix createCoefficientMatrix() const;

  void addConstantConstraint(Index _knot, Index _derivative, const OutputVector& _value);
  void addContinuityConstraint(Index _knot, Index _derivative);

  void fit();

  // These belong on a "Spline" class.
  Index getNumKnots() const;
  Index getNumOutputs() const;
  Scalar getDuration() const;
  Index getSegmentIndex(Scalar _t) const;
  OutputVector interpolate(Scalar _t, Index _derivative) const;

private:
  Index mNumKnots;
  Index mNumSegments;
  Index mNumCoefficients;
  Index mNumOutputs;
  Index mDimension;

  CoefficientMatrix mCoefficientMatrix;

  Index mRowIndex;
  TimeVector mTimes;
  ProblemMatrix mA;
  ProblemVector mB;

  std::vector<SolutionMatrix,
    Eigen::aligned_allocator<SolutionMatrix> > mSolution; // length _NumSegments

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(
       CoefficientMatrix::NeedsToAlign
    || TimeVector::NeedsToAlign
    || ProblemMatrix::NeedsToAlign
    || ProblemVector::NeedsToAlign
  );
};

} // namespace path
} // namespace r3

#include "detail/Spline-impl.h"

#endif // ifndef R3_PATH_SPLINE_H_
