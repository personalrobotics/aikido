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

/// An arbitrary dimensional polynomial spline. The number of coefficients,
/// outputs, and knot points may be specified either at compile time (via
/// template parameters) or at runtime (if the template parameters are
/// \c Eigen::Dynamic). We suggest setting as many of these parameters at
/// compile time as possible for best performance.
///
/// Note that this spline is not guaranteed to be continuous at knot points;
/// this is the responsbility of the user who constructs the splines
/// coefficients. See \c SplineProblem for a helper class that constructs a
/// continuous spline from a set of constraints.
///
/// \tparam _Scalar floating type used to represent a scalar value
/// \tparam _Index integral type used to represent an index
/// \tparam _NumCoefficients number of polynomial coefficients, or \c Dynamic
/// \tparam _NumOutputs number of outputs, or \c Dynamic
/// \tparam _NumKnots number of knots, or \c Dynamic
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
    = (NumSegmentsAtCompileTime != Eigen::Dynamic
        && _NumCoefficients != Eigen::Dynamic)
      ? (NumSegmentsAtCompileTime * NumCoefficientsAtCompileTime)
      : Eigen::Dynamic;

  using TimeVector = Eigen::Matrix<Scalar, NumKnotsAtCompileTime, 1>;
  using SolutionMatrix = Eigen::Matrix<
    Scalar, NumOutputsAtCompileTime, NumCoefficientsAtCompileTime>;
  using OutputVector = Eigen::Matrix<Scalar, NumOutputsAtCompileTime, 1>;
  using SolutionMatrices = std::vector<SolutionMatrix,
    Eigen::aligned_allocator<SolutionMatrix> >;

  /// Constructs an empty spline.
  SplineND() = default;

  /// Constructs a spline with the specified coefficients. The \c _solution
  /// is a vector of \c _times.size() - 1 spline coefficients, where the
  /// i-th matrix defines the polynomial coefficients for the segment between
  /// knot points (i) and (i + 1). Each coefficient matrix of size
  /// (num outputs) x (num coefficients), where element (i, j) is the
  /// coefficient on the \c x^j term for output \c i.
  ///
  /// \param _times times of knot points, must be monotone increasing
  /// \param _solution list of polynomial coefficients for each segment
  SplineND(
    const TimeVector& _times,
    const std::vector<SolutionMatrix,
      Eigen::aligned_allocator<SolutionMatrix> > &_solution);

  // Default copy and move semantics.
  SplineND(SplineND&& _other) = default;
  SplineND(const SplineND& _other) = default;
  SplineND& operator =(SplineND&& _other) = default;
  SplineND& operator =(const SplineND& _other) = default;

  /// Sets the time of the \c _index-th knot point. Times must remain monotone
  /// after performing this operation.
  ///
  /// \param _index index of a knot point
  /// \param _t new time for that knot point
  void setTime(Index _index, Scalar _t);

  /// Sets the times of all knot points.
  ///
  /// \param _index index of a knot point.
  /// \param _t new times, must be monotone increasing
  void setTimes(TimeVector &&_t);

  /// Sets the times of all knot points.
  ///
  /// \param _index index of a knot point.
  /// \param _t new times, must be monotone increasing
  void setTimes(const TimeVector &_t);

  /// Gets times of all knot points.
  ///
  /// \return times of knot points
  const TimeVector &getTimes() const;

  /// Gets polynomial coefficients for all segments.
  ///
  /// \return polynomial coefficients
  const SolutionMatrices &getCoefficients() const;

  /// Gets the number of knot points.
  ///
  /// \return number of knot points
  Index getNumKnots() const;

  /// Gets the number of outputs points.
  ///
  /// \return number of outputs points
  Index getNumOutputs() const;

  /// Gets an upperbound on the number of non-zero derivatives.
  ///
  /// \return upper bound on the number of non-zero derivatives
  Index getNumDerivatives() const;

  /// Gets the number of polynomial coefficients in each segment.
  ///
  /// \return number of polynomial coefficients
  Index getNumCoefficients() const;

  /// Gets the duration of the spline. This is the difference between the time
  /// of the first and last knot points.
  ///
  /// \return duration of the spline
  Scalar getDuration() const;

  /// Gets the index of the segment that contains time \c _t.
  ///
  /// \param _t time parameter
  /// \return index of the segment that contains this time
  Index getSegmentIndex(Scalar _t) const;

  /// Evaluate the \c _derivative-th order of the spline at time \c _t. The
  /// zero-th order derivative is function value. All derivatives greater than
  /// \c getNumDerivatives() are guaranteed to be zero.
  ///
  /// \param _t time parameter
  /// \param _derivative order of derivative to evaluate
  /// \return value of the specified derivative at time \c _t
  OutputVector evaluate(Scalar _t, Index _derivative = 0) const;

private:
  using CoefficientVector = Eigen::Matrix<
    Scalar, NumCoefficientsAtCompileTime, 1>;
  using CoefficientMatrix = Eigen::Matrix<
    Scalar, NumCoefficientsAtCompileTime, NumCoefficientsAtCompileTime>;

  TimeVector mTimes;
  SolutionMatrices mSolution;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(TimeVector::NeedsToAlign);
};


/// Utility for fitting splines given constraints on function value, derivative
/// value, and continuity. This class is intended to be used by calling methods
/// to add constraints to the spline, then calling \c fit() to find spline
/// coefficients that satisfy those constraints.
///
/// The number of coefficients, outputs, and knot points may be specified
/// either at compile time (via template parameters) or at runtime (if the
/// template parameters are \c Eigen::Dynamic). We suggest setting as many of
/// these parameters at compile time as possible for best performance.
///
/// \tparam _Scalar floating type used to represent a scalar value
/// \tparam _Index integral type used to represent an index
/// \tparam _NumCoefficients number of polynomial coefficients, or \c Dynamic
/// \tparam _NumOutputs number of outputs, or \c Dynamic
/// \tparam _NumKnots number of knots, or \c Dynamic
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
    = (NumSegmentsAtCompileTime != Eigen::Dynamic
          && _NumCoefficients != Eigen::Dynamic)
      ? (NumSegmentsAtCompileTime * NumCoefficientsAtCompileTime)
      : Eigen::Dynamic;

  using TimeVector = Eigen::Matrix<Scalar, NumKnotsAtCompileTime, 1>;
  using OutputVector = Eigen::Matrix<Scalar, NumOutputsAtCompileTime, 1>;
  using OutputMatrix = Eigen::Matrix<
    Scalar, NumCoefficientsAtCompileTime, NumOutputsAtCompileTime>;
  using CoefficientVector = Eigen::Matrix<
    Scalar, NumCoefficientsAtCompileTime, 1>;
  using CoefficientMatrix = Eigen::Matrix<
    Scalar, NumCoefficientsAtCompileTime, NumCoefficientsAtCompileTime>;
  using ProblemMatrix = Eigen::SparseMatrix<Scalar, 0, Index>;
  using ProblemVector = Eigen::Matrix<
    Scalar, DimensionAtCompileTime, NumOutputsAtCompileTime>;
  using SolutionMatrix = Eigen::Matrix<
    Scalar, NumOutputsAtCompileTime, NumCoefficientsAtCompileTime>;
  using Spline = SplineND<
    Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>;

  /// Constructs a spline fitting problem with the knot points at the specified
  /// times. This overload is only supported if \c _NumCoefficients and
  /// \c _NumOutputs are specified as template parameters. The length of
  /// \c _times must equal \c _NumKnots if that template parameter is specified
  ///
  /// If \c _NumCoefficients or \c _NumOutputs is \c Eigen::Dynamic, you must
  /// use the other constructor overload.
  ///
  /// \param _times list of knot point times, must be monotone increasing
  explicit SplineProblem(const TimeVector& _times);

  /// Constructs a spline fitting problem with knot points at the specified
  /// times and the specified number of coefficients and outputs. These
  /// parameters must match \c _NumKnots and \c _NumCoefficients if they are
  /// specified at compile time. Additionally, the length of \c _times must
  /// match \c _NumKnots if that template parameter is specified.
  ///
  /// \param _times list of knot point times, must be monotone increasing
  /// \param _numCoefficients number of polynomial coefficients
  /// \param _numOutputs number of outputs
  SplineProblem(
    const TimeVector& _times, Index _numCoefficients, Index _numOutputs);

  // Default copy and move semantics.
  SplineProblem(SplineProblem&& _other) = default;
  SplineProblem(const SplineProblem& _other) = default;
  SplineProblem& operator =(SplineProblem&& _other) = default;
  SplineProblem& operator =(const SplineProblem& _other) = default;

  /// Creates a vector of the form [ 1, t, t^2, ... t^_n ] 
  static CoefficientVector createTimeVector(Scalar _t, Index _i, Index _n);

  /// Creates the \c _n by \c _n matrix of derivative coefficients for a
  /// polynomial with \c _n coefficients. The i-th row of this matrix
  /// corresponds to coefficients for the i-th derivative. The j-th column of
  /// this matrix corresponds to the coefficient on x^j.
  ///
  /// \param _n number of coefficients
  /// \return polynomial coefficient matrix
  static CoefficientMatrix createCoefficientMatrix(Index _n);

  /// Adds a constraint that the \c _derivative-th order derivative of knot
  /// point \c _knot should equal \c _value. This adds one constraint if the
  /// knot is an end point and two constraints for interior knots. The zero-th
  /// derivative is function value.
  ///
  /// \param _knot knot index to constraint
  /// \param _derivative order of derivative to constraint
  /// \param _value desired value of that knot point's derivative
  void addConstantConstraint(
    Index _knot, Index _derivative, const OutputVector& _value);

  /// Adds a continuity constraint on the \c _derivative-th order derivative at
  /// knot point \c _knot. This operation is only defined for interior knot
  /// points and adds one constraint.
  ///
  /// \param _knot knot index to constraint
  /// \param _derivative order of derivative to constraint
  void addContinuityConstraint(Index _knot, Index _derivative);

  /// Fit a spline given the constraints on added to this object. The behavior 
  /// of this function is undefined if the problem is over or
  /// under-constrained. To avoid this, be sure to only add
  /// (num coefficients) * (num knots - 1) constraints to this class.
  ///
  /// \return spline that satisfies the constraints
  Spline fit();

  /// Gets the number of knot points.
  ///
  /// \return number of knot points
  Index getNumKnots() const;

  /// Gets the number of outputs points.
  ///
  /// \return number of outputs points
  Index getNumOutputs() const;

  /// Gets the duration of the spline. This is the difference between the time
  /// of the first and last knot points.
  ///
  /// \return duration of the spline
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

#include "detail/Spline-impl.hpp"

#endif // AIKIDO_PATH_SPLINE_H_
