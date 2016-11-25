#ifndef AIKIDO_TESTS_EIGEN_TESTS_HPP_
#define AIKIDO_TESTS_EIGEN_TESTS_HPP_
#include <Eigen/Dense>
#include <gtest/gtest.h>

#define ASSERT_EIGEN_EQUAL(_expected_, _actual_, _epsilon_)\
ASSERT_TRUE(aikido::tests::CompareEigenMatrices(\
  _expected_, _actual_, _epsilon_))

#define EXPECT_EIGEN_EQUAL(_expected_, _actual_, _epsilon_)\
EXPECT_TRUE(aikido::tests::CompareEigenMatrices(\
  _expected_, _actual_, _epsilon_))

namespace aikido {
namespace tests {

template <class Derived1, class Derived2>
testing::AssertionResult CompareEigenMatrices(
  const Eigen::MatrixBase<Derived1>& _expected,
  const Eigen::MatrixBase<Derived2>& _actual,
  double _epsilon)
{
  static_assert(
    std::is_same<typename Eigen::MatrixBase<Derived1>::Index,
                 typename Eigen::MatrixBase<Derived2>::Index>::value,
    "Matrices have different Index types.");
  static_assert(
    std::is_same<typename Eigen::MatrixBase<Derived1>::Scalar,
                 typename Eigen::MatrixBase<Derived2>::Scalar>::value,
     "Matrices have different Scalar types.");

  using Index = typename Eigen::MatrixBase<Derived1>::Index;
  using Scalar = typename Eigen::MatrixBase<Derived2>::Scalar;

  if (_actual.rows() != _expected.rows())
    return testing::AssertionFailure()
      << "Matrices have different sizes: expected " << _expected.rows()
      << " rows, got " << _actual.rows() << ".";

  if (_actual.cols() != _expected.cols())
    return testing::AssertionFailure()
      << "Matrices have different sizes: expected " << _expected.cols()
      << " columns, got " << _actual.cols() << ".";

  for (Index irow = 0; irow < _expected.rows(); ++irow)
  for (Index icol = 0; icol < _expected.cols(); ++icol)
  {
    const Scalar actualValue = _actual(irow, icol);
    const Scalar expectedValue = _expected(irow, icol);
    const Scalar errorValue = std::abs(actualValue - expectedValue);

    if (errorValue > _epsilon)
      return testing::AssertionFailure()
        << "Matrices differ in row " << irow << ", column " << icol << ": "
        << std::setprecision(std::numeric_limits<Scalar>::max_digits10)
        << expectedValue << " !=: " << actualValue << ".";
  }
  return testing::AssertionSuccess();
}

inline Eigen::VectorXd make_vector(double _value)
{
  Eigen::Matrix<double, 1, 1> valueVector;
  valueVector << _value;
  return valueVector;
}

inline Eigen::VectorXd make_vector(double _value1, double _value2)
{
  Eigen::Matrix<double, 2, 1> valueVector;
  valueVector << _value1, _value2;
  return valueVector;
}

} // namespace tests
} // namespace aikido

#endif // ifndef AIKIDO_TESTS_EIGEN_TESTS_HPP_
