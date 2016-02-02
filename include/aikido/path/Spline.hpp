#ifndef AIKIDO_PATH_SPLINE_H_
#define AIKIDO_PATH_SPLINE_H_

#include <cstddef>
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Sparse>
#include <Eigen/StdVector>

namespace aikido {
namespace path {

template <
  class _Scalar = double,
  class _Index = int,
  _Index _NumCoefficients = Eigen::Dynamic,
  _Index _NumOutputs = Eigen::Dynamic,
  _Index _NumKnots = Eigen::Dynamic>
class SplineND
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
  using SolutionMatrix = Eigen::Matrix<Scalar, NumOutputsAtCompileTime, NumCoefficientsAtCompileTime>;
  using OutputVector = Eigen::Matrix<Scalar, NumOutputsAtCompileTime, 1>;
  using SolutionMatrices = std::vector<SolutionMatrix,
    Eigen::aligned_allocator<SolutionMatrix> >;

  SplineND() = default;
  SplineND(
    const TimeVector& _times,
    const std::vector<SolutionMatrix,
      Eigen::aligned_allocator<SolutionMatrix> > &_solution);

  // Default copy and move semantics.
  SplineND(SplineND&& _other) = default;
  SplineND(const SplineND& _other) = default;
  SplineND& operator =(SplineND&& _other) = default;
  SplineND& operator =(const SplineND& _other) = default;

  void setTime(Index _index, Scalar _t);
  void setTimes(TimeVector &&_t);
  void setTimes(const TimeVector &_t);
  const TimeVector &getTimes() const;

  const SolutionMatrices &getCoefficients() const;

  Index getNumKnots() const;
  Index getNumOutputs() const;
  Index getNumDerivatives() const;
  Index getNumCoefficients() const;

  Scalar getDuration() const;

  Index getSegmentIndex(Scalar _t) const;
  OutputVector evaluate(Scalar _t, Index _derivative = 0) const;

private:
  using CoefficientVector = Eigen::Matrix<Scalar, NumCoefficientsAtCompileTime, 1>;
  using CoefficientMatrix = Eigen::Matrix<Scalar, NumCoefficientsAtCompileTime, NumCoefficientsAtCompileTime>;

  TimeVector mTimes;
  SolutionMatrices mSolution; // length _NumSegments

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(TimeVector::NeedsToAlign);
};

template <
  class _Scalar = double,
  class _Index = int,
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
  using ProblemMatrix = Eigen::SparseMatrix<Scalar, 0, Index>;
  using ProblemVector = Eigen::Matrix<Scalar, DimensionAtCompileTime, NumOutputsAtCompileTime>;
  using SolutionMatrix = Eigen::Matrix<Scalar, NumOutputsAtCompileTime, NumCoefficientsAtCompileTime>;
  using Spline = SplineND<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>;

  explicit SplineProblem(const TimeVector& _times);
  SplineProblem(const TimeVector& _times, Index _numCoefficients, Index _numOutputs);

  // Default copy and move semantics.
  SplineProblem(SplineProblem&& _other) = default;
  SplineProblem(const SplineProblem& _other) = default;
  SplineProblem& operator =(SplineProblem&& _other) = default;
  SplineProblem& operator =(const SplineProblem& _other) = default;

  static CoefficientVector createTimeVector(Scalar _t, Index _i, Index _n);
  static CoefficientMatrix createCoefficientMatrix(Index _n);

  void addConstantConstraint(Index _knot, Index _derivative, const OutputVector& _value);
  void addContinuityConstraint(Index _knot, Index _derivative);

  Spline fit();

  // These belong on a "Spline" class.
  Index getNumKnots() const;
  Index getNumOutputs() const;
  Scalar getDuration() const;

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
} // namespace aikido

#include "detail/Spline-impl.h"

#include "Trajectory.h"
#include "SplineTrajectory.h"

#endif // AIKIDO_PATH_SPLINE_H_
