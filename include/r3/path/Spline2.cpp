#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/QR>

template <
  class Scalar = double,
  class Index = ptrdiff_t,
  Index _NumCoefficients = Eigen::Dynamic,
  Index _NumOutputs = Eigen::Dynamic,
  Index _NumKnots = Eigen::Dynamic>
class SplineProblem {
public:
  static constexpr Index _NumSegments
    = (_NumKnots != Eigen::Dynamic) ? (_NumKnots - 1) : Eigen::Dynamic;
  static constexpr Index _Dimension
    = (_NumSegments != Eigen::Dynamic && _NumCoefficients != Eigen::Dynamic)
      ? (_NumSegments * _NumCoefficients) : Eigen::Dynamic;

  using TimeVector = Eigen::Matrix<Scalar, _NumKnots, 1>;
  using OutputVector = Eigen::Matrix<Scalar, _NumOutputs, 1>;
  using OutputMatrix = Eigen::Matrix<Scalar, _NumCoefficients, _NumOutputs>;
  using CoefficientVector = Eigen::Matrix<Scalar, _NumCoefficients, 1>;
  using CoefficientMatrix = Eigen::Matrix<Scalar, _NumCoefficients, _NumCoefficients>;

  SplineProblem(const TimeVector& _times, Index _numCoefficients, Index _numOutputs);

  CoefficientVector createTimeVector(Scalar _t, Index _i) const;
  CoefficientMatrix createTimeMatrix(Scalar _t) const;
  CoefficientMatrix createCoefficientMatrix() const;

  void addConstantConstraint(Index _knot, Index _derivative, const OutputVector& _value);
  void addEqualityConstraint(Index _knot1, Index _knot2, Index _derivative);

  void fit();
  Index getSegmentIndex(Scalar _t) const;
  OutputVector interpolate(Scalar _t, Index _derivative) const;

//private:
  using SolutionMatrix = Eigen::Matrix<Scalar, _NumSegments, _NumCoefficients>;

  Index mNumKnots;
  Index mNumSegments;
  Index mNumCoefficients;
  Index mNumOutputs;
  Index mDimension;

  Eigen::Matrix<Scalar, _NumCoefficients, _NumCoefficients> mCoefficientMatrix;

  Index mRowIndex;
  TimeVector mTimes;
  Eigen::Matrix<Scalar, _Dimension, _Dimension> mA;
  Eigen::Matrix<Scalar, _Dimension, _NumOutputs> mB;

  std::vector<SolutionMatrix> mSolution;
};

// ---

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
      mSolution(mNumSegments, SolutionMatrix(mNumSegments, _numCoefficients))
{
  mA.setZero();
  mB.setZero();

  std::cout << "\n\n"
            << "mNumKnots = " << mNumKnots << "\n"
            << "mNumSegments = " << mNumSegments << "\n"
            << "mNumCoefficients = " << mNumCoefficients << "\n"
            << "mNumOutputs = " << mNumOutputs << "\n"
            << "mDimension = " << mDimension << "\n"
            << "\n\n"
            << std::flush;

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
  ::addEqualityConstraint(Index _knot1, Index _knot2, Index _derivative)
{
  assert(0 <= _knot1 && _knot1 < mNumKnots);
  assert(0 <= _knot2 && _knot2 < mNumKnots);
  assert(_knot1 != _knot2);
  assert(0 <= _derivative && _derivative < mNumCoefficients);
  assert(mRowIndex < mDimension);

  const CoefficientVector derivativeVector = mCoefficientMatrix.row(_derivative);
  const CoefficientVector timeVector1 = createTimeVector(mTimes[_knot1], _derivative);
  const CoefficientVector timeVector2 = createTimeVector(mTimes[_knot2], _derivative);
  
  mA.block(mRowIndex, _knot1 * mNumCoefficients, 1, mNumCoefficients)
    = derivativeVector.cwiseProduct(timeVector1);
  mA.block(mRowIndex, _knot2 * mNumCoefficients, 1, mNumCoefficients)
    = -derivativeVector.cwiseProduct(timeVector2);
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
  using MatrixType = Eigen::Matrix<Scalar, _Dimension, _Dimension>;

  // Perform the QR decomposition once. 
  Eigen::HouseholderQR<MatrixType> solver = mA.householderQr();

  for (Index ioutput = 0; ioutput < mNumOutputs; ++ioutput) {
    // Solve for the spline coefficients for each output dimension.
    Eigen::Matrix<Scalar, _Dimension, 1> solutionVector
      = solver.solve(mB.col(ioutput));

    // Split the coefficients by segment.
    OutputMatrix& solutionMatrix = mSolution[ioutput];
    for (Index isegment = 0; isegment < mNumSegments; ++isegment) {
      solutionMatrix.row(isegment) = solutionVector.segment(
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
auto SplineProblem<Scalar, Index, _NumCoefficients, _NumOutputs, _NumKnots>
  ::interpolate(Scalar _t, Index _derivative) const -> OutputVector
{
  const CoefficientVector timeVector = createTimeVector(_t, _derivative);
  const CoefficientVector derivativeVector = mCoefficientMatrix.row(_derivative);
  const CoefficientVector evaluationVector = derivativeVector.cwiseProduct(timeVector);
  const Index segmentIndex = getSegmentIndex(_t);

  OutputVector output(mNumOutputs);

  for (Index ioutput = 0; ioutput < mNumOutputs; ++ioutput) {
    const CoefficientVector solutionVector = mSolution[ioutput].row(segmentIndex);
    output[ioutput] = evaluationVector.dot(solutionVector);
  }

  return output;
}

// ---

int main(int argc, char **argv)
{
  using Eigen::VectorXd;

  using Vector1d = Eigen::Matrix<double, 1, 1>;

  auto Value = [](double x) {
    Eigen::Matrix<double, 1, 1> v;
    v << x;
    return v;
  };

  VectorXd times(3);
  times << 0, 1, 3;

  SplineProblem<> problem(times, 2, 1);
  problem.addConstantConstraint(0, 0, Value(5));
  problem.addConstantConstraint(1, 0, Value(6));
  problem.addConstantConstraint(2, 0, Value(0));

  problem.fit();

  for (double t = 0; t <= 2; t += 0.05) {
    std::cout << t << '\t' << problem.interpolate(t, 0) << '\n';
  }

  return 0;
}
