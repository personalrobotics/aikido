#include <cstdlib>
#include <gtest/gtest.h>
#include <aikido/common/Spline.hpp>
#include "eigen_tests.hpp"

using namespace aikido::tests;

static constexpr double EPSILON = 1e-6;

class SplineProblemTests : public testing::Test
{
protected:
  using SplineProblem = aikido::common::SplineProblem<>;

  void SetUp() override
  {
  }
};

TEST_F(SplineProblemTests, fit_ConstantConstraints)
{
  SplineProblem::TimeVector times(3);
  times << 0., 1., 2.;

  SplineProblem problem(times, 4, 2);
  problem.addConstantConstraint(0, 0, make_vector(1., 2.));
  problem.addConstantConstraint(1, 0, make_vector(2., 3.));
  problem.addConstantConstraint(2, 0, make_vector(3., 4.));

  problem.addConstantConstraint(0, 1, make_vector(0., 0.));
  problem.addConstantConstraint(1, 1, make_vector(1., 1.));
  problem.addConstantConstraint(2, 1, make_vector(0., 0.));

  SplineProblem::Spline spline = problem.fit();
  EXPECT_EIGEN_EQUAL(times, spline.getTimes(), EPSILON);

  Eigen::Matrix<double, 2, 4> coefficients1;
  coefficients1 << 1., 0., 2., -1., 
                   2., 0., 2., -1.;
  Eigen::Matrix<double, 2, 4> coefficients2;
  coefficients2 << 3., -4., 4., -1.,
                   4., -4., 4., -1.;

  EXPECT_EIGEN_EQUAL(coefficients1, spline.getCoefficients()[0], EPSILON);
  EXPECT_EIGEN_EQUAL(coefficients2, spline.getCoefficients()[1], EPSILON);
}

TEST_F(SplineProblemTests, fit_ContinuityConstraints)
{

  SplineProblem::TimeVector times(3);
  times << 0., 1., 2.;

  SplineProblem problem(times, 4, 2);
  problem.addConstantConstraint(0, 0, make_vector(1., 2.));
  problem.addConstantConstraint(2, 0, make_vector(2., 3.));
  problem.addConstantConstraint(2, 0, make_vector(3., 4.));

  problem.addConstantConstraint(0, 1, make_vector(0., 0.));
  problem.addConstantConstraint(1, 1, make_vector(1., 1.));

  problem.addContinuityConstraint(1, 1);
  problem.addContinuityConstraint(1, 2);

  SplineProblem::Spline spline = problem.fit();
  EXPECT_EIGEN_EQUAL(times, spline.getTimes(), EPSILON);

  // TODO: Check the coefficient matrices.
#if  0
  EXPECT_EIGEN_EQUAL(coefficients1, spline.getCoefficients()[0], EPSILON);
  EXPECT_EIGEN_EQUAL(coefficients2, spline.getCoefficients()[1], EPSILON);
#endif
}
