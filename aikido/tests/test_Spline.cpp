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
  void SetUp() override
  {
  }
};

TEST_F(SplineNDTests, LinearSpline)
{
  using LinearSpline = SplineND<double, int, 2>;

  LinearSpline::TimeVector times(3);
  times << 0., 1., 2.;

  LinearSpline::SolutionMatrices coefficients(2,
    LinearSpline::SolutionMatrix(1, 2));
  coefficients[0] << 0.,  1.;
  coefficients[1] << 3., -2.;

  LinearSpline spline(times, coefficients);

  EXPECT_EIGEN_EQUAL(times, spline.getTimes(), EPSILON);
  ASSERT_EQ(2, spline.getCoefficients().size());
  EXPECT_EIGEN_EQUAL(coefficients[0], spline.getCoefficients()[0], EPSILON);
  EXPECT_EIGEN_EQUAL(coefficients[1], spline.getCoefficients()[1], EPSILON);

  EXPECT_EQ(3, spline.getNumKnots());
  EXPECT_EQ(1, spline.getNumOutputs());
  EXPECT_EQ(1, spline.getNumDerivatives());
  EXPECT_EQ(2, spline.getNumCoefficients());
  EXPECT_DOUBLE_EQ(2., spline.getDuration());

  EXPECT_EQ(0, spline.getSegmentIndex(-0.01));
  EXPECT_EQ(0, spline.getSegmentIndex(0.));
  EXPECT_EQ(0, spline.getSegmentIndex(0.5));
  EXPECT_EQ(1, spline.getSegmentIndex(1.5));
  EXPECT_EQ(1, spline.getSegmentIndex(2.01));

  EXPECT_EIGEN_EQUAL(make_singleton( 0.0), spline.evaluate(0.0, 0), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton( 0.5), spline.evaluate(0.5, 0), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton( 0.0), spline.evaluate(1.5, 0), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton(-1.0), spline.evaluate(2.0, 0), EPSILON);

  EXPECT_EIGEN_EQUAL(make_singleton( 1.), spline.evaluate(0.5, 1), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton(-2.), spline.evaluate(1.5, 1), EPSILON);

  EXPECT_EIGEN_EQUAL(make_singleton(0.), spline.evaluate(0.5, 2), EPSILON);
  EXPECT_EIGEN_EQUAL(make_singleton(0.), spline.evaluate(1.5, 2), EPSILON);
}
