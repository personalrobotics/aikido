#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace r3 {
namespace path {

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::SplineProblem(const TimeVector& _times)
    : SplineProblem(_times, NumCoefficientsAtCompileTime, NumOutputsAtCompileTime)
{
  static_assert(NumCoefficientsAtCompileTime != Eigen::Dynamic,
    "NumCoefficientsAtCompileTime must be static to use this constructor.");
  static_assert(NumOutputsAtCompileTime != Eigen::Dynamic,
    "NumOutputsAtCompileTime must be static to use this constructor.");
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::SplineProblem(const TimeVector& _times, Index _numCoefficients, Index _numOutputs)
    : mNumKnots(_times.size()),
      mNumSegments(std::max<Index>(mNumKnots - 1, 0)),
      mNumCoefficients(_numCoefficients),
      mNumOutputs(_numOutputs),
      mDimension(mNumSegments * _numCoefficients),
      mCoefficientMatrix(createCoefficientMatrix()),
      mRowIndex(0),
      mTimes(_times),
      mA(mDimension, mDimension),
      mB(mDimension, _numOutputs),
      mSolution(mNumSegments, SolutionMatrix(_numOutputs, _numCoefficients))
{
  mA.setZero();
  mB.setZero();

  if (!std::is_sorted(mTimes.data(), mTimes.data() + mTimes.size())) {
    throw std::runtime_error("Times are not monotonically increasing.");
  }
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
void SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::addConstantConstraint(
    Index _knot, Index _derivative, const OutputVector& _value)
{
  assert(0 <= _knot && _knot < mNumKnots);
  assert(0 <= _derivative && _derivative < mNumCoefficients);
  assert(_value.size() == mNumOutputs);

  const CoefficientVector timeVector = createTimeVector(mTimes[_knot], _derivative);
  const CoefficientVector derivativeVector = mCoefficientMatrix.row(_derivative);
  const CoefficientVector coeffVector = derivativeVector.cwiseProduct(timeVector);

  // Position constraint on segment before this knot.
  if (_knot > 0) {
    assert(mRowIndex < mDimension);

    mA.block(mRowIndex, (_knot - 1) * mNumCoefficients, 1, mNumCoefficients)
      = coeffVector.transpose();
    mB.row(mRowIndex) = _value;

    ++mRowIndex;
  }

  // Position constraint on segment after this knot.
  if (_knot + 1 < mNumKnots) {
    assert(mRowIndex < mDimension);

    mA.block(mRowIndex, _knot * mNumCoefficients, 1, mNumCoefficients)
      = coeffVector.transpose();
    mB.row(mRowIndex) = _value;

    ++mRowIndex;
  }
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
void SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::addContinuityConstraint(Index _knot, Index _derivative)
{
  assert(0 <= _knot && _knot < mNumKnots);
  assert(_knot != 0 && _knot + 1 != mNumKnots);
  assert(0 <= _derivative && _derivative < mNumCoefficients);
  assert(mRowIndex < mDimension);

  const CoefficientVector derivativeVector = mCoefficientMatrix.row(_derivative);
  const CoefficientVector timeVector = createTimeVector(mTimes[_knot], _derivative);
  const CoefficientVector coeffVector = derivativeVector.cwiseProduct(timeVector);
  
  mA.block(mRowIndex, (_knot - 1) * mNumCoefficients, 1, mNumCoefficients)
    = coeffVector.transpose();
  mA.block(mRowIndex,  _knot      * mNumCoefficients, 1, mNumCoefficients)
    = -coeffVector.transpose();
  mB.row(mRowIndex).setZero();

  ++mRowIndex;
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
auto SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::createTimeVector(Scalar _t, Index _i) const -> CoefficientVector
{
  CoefficientVector exponents(mNumCoefficients);

  for (Index j = 0; j < mNumCoefficients; ++j) {
    if (j > _i) {
      exponents[j] = std::pow(_t, j - _i);
    } else if (j == _i) {
      exponents[j] = 1;
    } else {
      exponents[j] = 0;
    }
  }

  return exponents;
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
void SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::fit()
{
  assert(mRowIndex == mDimension);

  using MatrixType = Eigen::Matrix<Scalar, DimensionAtCompileTime, DimensionAtCompileTime>;

  // Perform the QR decomposition once. 
  Eigen::HouseholderQR<MatrixType> solver = mA.householderQr();

  for (Index ioutput = 0; ioutput < mNumOutputs; ++ioutput) {
    // Solve for the spline coefficients for each output dimension.
    Eigen::Matrix<Scalar, DimensionAtCompileTime, 1> solutionVector
      = solver.solve(mB.col(ioutput));

    // Split the coefficients by segment.
    for (Index isegment = 0; isegment < mNumSegments; ++isegment) {
      SolutionMatrix& solutionMatrix = mSolution[isegment];
      solutionMatrix.row(ioutput) = solutionVector.segment(
        isegment * mNumCoefficients, mNumCoefficients);
    }
  }
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
auto SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::createCoefficientMatrix() const -> CoefficientMatrix
{
  CoefficientMatrix coefficients(mNumCoefficients, mNumCoefficients);
  coefficients.setZero();

  if (mNumCoefficients > 0) {
    coefficients.row(0).setOnes();
  }

  for (Index i = 1; i < mNumCoefficients; ++i) {
    for (Index j = i; j < mNumCoefficients; ++j) {
      coefficients(i, j) = (j - i + 1) * coefficients(i - 1, j);
    }
  }
  return coefficients;
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
auto SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::getSegmentIndex(Scalar _t) const -> Index
{
  if (_t <= mTimes[0]) {
    return 0;
  } else if (_t >= mTimes[mNumKnots - 1]) {
    return mNumSegments - 1;
  } else {
    auto it = std::lower_bound(mTimes.data(), mTimes.data() + mTimes.size(), _t);
    return it - mTimes.data() - 1;
  }
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
Index SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::getNumKnots() const
{
  return mNumKnots;
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
Index SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::getNumOutputs() const
{
  return mNumOutputs;
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
Scalar SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::getDuration() const
{
  if (mTimes.size() > 0) {
    return mTimes[mTimes.size() - 1];
  } else {
    return 0;
  }
}

template <
  class Scalar, class Index,
  Index _NumCoefficients, Index _NumOutputs, Index _NumKnots>
auto SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::interpolate(Scalar _t, Index _derivative) const -> OutputVector
{
  const CoefficientVector timeVector = createTimeVector(_t, _derivative);
  const CoefficientVector derivativeVector = mCoefficientMatrix.row(_derivative);
  const CoefficientVector evaluationVector = derivativeVector.cwiseProduct(timeVector);
  const Index segmentIndex = getSegmentIndex(_t);
  const SolutionMatrix& solutionMatrix = mSolution[segmentIndex];

  OutputVector output(mNumOutputs);

  for (Index ioutput = 0; ioutput < mNumOutputs; ++ioutput) {
    const CoefficientVector solutionVector = solutionMatrix.row(ioutput);
    output[ioutput] = evaluationVector.dot(solutionVector);
  }

  return output;
}

} // namespace path
} // namespace r3
