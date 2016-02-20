#include <cstdlib>
#include <gtest/gtest.h>
#include <aikido/path/Spline.hpp>

#define ASSERT_EIGEN_EQUAL(_expected_, _actual_, _epsilon_)\
ASSERT_TRUE(CompareEigenMatrices(_expected_, _actual_, _epsilon_))

#define EXPECT_EIGEN_EQUAL(_expected_, _actual_, _epsilon_)\
EXPECT_TRUE(CompareEigenMatrices(_expected_, _actual_, _epsilon_))

using aikido::path::SplineND;

static constexpr double EPSILON = 1e-6;

namespace {

template <class Derived>
testing::AssertionResult CompareEigenMatrices(
  const Eigen::MatrixBase<Derived>& _expected,
  const Eigen::MatrixBase<Derived>& _actual,
  double _epsilon)
{
  using Index = typename Eigen::ArrayBase<Derived>::Index;
  using Scalar = typename Eigen::ArrayBase<Derived>::Scalar;

  if (_actual.rows() != _expected.rows())
    return testing::AssertionFailure()
      << "Arrays have different sizes: expected " << _expected.rows()
      << " rows, got " << _actual.rows() << ".";

  if (_actual.cols() != _expected.cols())
    return testing::AssertionFailure()
      << "Arrays have different sizes: expected " << _expected.cols()
      << " columns, got " << _actual.cols() << ".";

  for (Index irow = 0; irow < _expected.rows(); ++irow)
  for (Index icol = 0; icol < _expected.cols(); ++icol)
  {
    const Scalar actualValue = _actual(irow, icol);
    const Scalar expectedValue = _expected(irow, icol);
    const Scalar errorValue = std::abs(actualValue - expectedValue);

    if (errorValue > _epsilon)
      return testing::AssertionFailure()
        << "Arrays differ in row " << irow << ", column " << icol << ": "
        << std::setprecision(std::numeric_limits<Scalar>::max_digits10)
        << expectedValue << " !=: " << actualValue << ".";
  }
  return testing::AssertionSuccess();
}

Eigen::VectorXd make_singleton(double _value)
{
  Eigen::Matrix<double, 1, 1> valueVector;
  valueVector << _value;
  return valueVector;
}

} // namespace

class SplineNDTests : public testing::Test
{
protected:
  using LinearSpline = SplineND<double, int, 2>;

  void SetUp() override
  {
    mTimesA.resize(3);
    mTimesA << 0., 1., 2.;

    mCoefficientsA.resize(2, LinearSpline::SolutionMatrix(1, 2));
    mCoefficientsA[0] << 0.,  1.;
    mCoefficientsA[1] << 3., -2.;

    mSpline = LinearSpline(mTimesA, mCoefficientsA);
  }

  LinearSpline mSpline;
  LinearSpline::TimeVector mTimesA;
  LinearSpline::SolutionMatrices mCoefficientsA;
};

TEST_F(SplineNDTests, LinearSpline_getTimes)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EIGEN_EQUAL(mTimesA, spline.getTimes(), EPSILON);
}

TEST_F(SplineNDTests, LinearSpline_getCoefficients)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  ASSERT_EQ(2, spline.getCoefficients().size());
  EXPECT_EIGEN_EQUAL(mCoefficientsA[0], spline.getCoefficients()[0], EPSILON);
  EXPECT_EIGEN_EQUAL(mCoefficientsA[1], spline.getCoefficients()[1], EPSILON);
}

TEST_F(SplineNDTests, LinearSpline_getNumKnots)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EQ(3, spline.getNumKnots());
}

TEST_F(SplineNDTests, LinearSpline_getNumOutputs)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EQ(1, spline.getNumOutputs());
}

TEST_F(SplineNDTests, LinearSpline_getNumDerivatives)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EQ(1, spline.getNumDerivatives());
}

TEST_F(SplineNDTests, LinearSpline_getNumCoefficients)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EQ(2, spline.getNumCoefficients());
}

TEST_F(SplineNDTests, LinearSpline_getDuration)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_DOUBLE_EQ(2., spline.getDuration());
}

TEST_F(SplineNDTests, LinearSpline_getSegmentIndex)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EQ(0, spline.getSegmentIndex(0.));
  EXPECT_EQ(0, spline.getSegmentIndex(0.5));
  EXPECT_EQ(1, spline.getSegmentIndex(1.5));
}

TEST_F(SplineNDTests, LinearSpline_getSegmentIndex_OutOfBounds)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EQ(0, spline.getSegmentIndex(-0.01));
  EXPECT_EQ(1, spline.getSegmentIndex(2.01));
}

TEST_F(SplineNDTests, LinearSpline_evaluate_Derivative0)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EIGEN_EQUAL(make_singleton( 0.0), spline.evaluate(0.0, 0), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton( 0.5), spline.evaluate(0.5, 0), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton( 0.0), spline.evaluate(1.5, 0), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton(-1.0), spline.evaluate(2.0, 0), EPSILON);
}

TEST_F(SplineNDTests, LinearSpline_evaluate_Derivative1)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EIGEN_EQUAL(make_singleton( 1.), spline.evaluate(0.5, 1), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton(-2.), spline.evaluate(1.5, 1), EPSILON);
}

TEST_F(SplineNDTests, LinearSpline_evaluate_Derivative2)
{
  LinearSpline spline(mTimesA, mCoefficientsA);

  EXPECT_EIGEN_EQUAL(make_singleton(0.), spline.evaluate(0.5, 2), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton(0.), spline.evaluate(1.5, 2), EPSILON);
}
