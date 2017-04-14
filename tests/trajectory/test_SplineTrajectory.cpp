#include <aikido/trajectory/Spline.hpp>
#include <aikido/statespace/Rn.hpp>
#include <gtest/gtest.h>

using namespace aikido::statespace;
using aikido::trajectory::Spline;
using Eigen::Matrix2d;
using Eigen::Vector2d;

static const Vector2d START_VALUE(1., 2.);

class SplineTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mStateSpace = std::make_shared<R2>();

    mStartState = static_cast<R2::State*>(mStateSpace->allocateState());
    mStateSpace->setValue(mStartState, START_VALUE);
  }

  void TearDown() override
  {
    mStateSpace->freeState(mStartState);
  }

  std::shared_ptr<R2> mStateSpace;
  R2::State* mStartState;
  std::shared_ptr<Spline> mTrajectory;
};

TEST_F(SplineTest, addSegment_NegativeDurationThrows)
{
  Eigen::Matrix2d coefficients;
  coefficients << 0., 1.,
                  0., 2.;

  Spline trajectory(mStateSpace, 0.);

  EXPECT_THROW({
    trajectory.addSegment(coefficients, -1., mStartState);
  }, std::invalid_argument);
}

TEST_F(SplineTest, addSegment_ZeroDurationThrows)
{
  Eigen::Matrix2d coefficients;
  coefficients << 0., 1.,
                  0., 2.;

  Spline trajectory(mStateSpace, 0.);

  EXPECT_THROW({
    trajectory.addSegment(coefficients, 0., mStartState);
  }, std::invalid_argument);
}

TEST_F(SplineTest, addSegment_IncorrectDimension_Throws)
{
  Eigen::Matrix<double, 1, 2> coefficients;
  coefficients << 0., 1.;

  Spline trajectory(mStateSpace, 0.);

  EXPECT_THROW({
    trajectory.addSegment(coefficients, 1., mStartState);
  }, std::invalid_argument);
}

TEST_F(SplineTest, addSegment_EmptyCoefficients_Throws)
{
  Eigen::Matrix<double, 2, 0> coefficients;

  Spline trajectory(mStateSpace, 0.);

  EXPECT_THROW({
    trajectory.addSegment(coefficients, 1., mStartState);
  }, std::invalid_argument);
}

TEST_F(SplineTest, getStateSpace)
{
  Spline trajectory(mStateSpace, 0.);

  EXPECT_EQ(mStateSpace, trajectory.getStateSpace());
}

TEST_F(SplineTest, getNumDerivatives_IsEmpty_ReturnsZero)
{
  Spline trajectory(mStateSpace, 0.);

  EXPECT_EQ(0, trajectory.getNumDerivatives());
}

TEST_F(SplineTest, getNumDerivatives_IsConstant_ReturnsZero)
{
  Spline trajectory(mStateSpace, 0.);
  trajectory.addSegment(Vector2d(4., 5.), 1., mStartState);

  EXPECT_EQ(0, trajectory.getNumDerivatives());
}

TEST_F(SplineTest, getNumDerivatives_IsLinear_ReturnsOne)
{
  Matrix2d coefficients;
  coefficients << 0., 1.,
                  0., 2.;

  Spline trajectory(mStateSpace, 0.);
  trajectory.addSegment(coefficients, 1., mStartState);

  EXPECT_EQ(1, trajectory.getNumDerivatives());
}

TEST_F(SplineTest, getNumDerivatives_IsQuadratic_ReturnsTwo)
{
  Eigen::Matrix<double, 2, 3> coefficients;
  coefficients << 0., 0., 1.,
                  0., 0., 2.;

  Spline trajectory(mStateSpace, 0.);
  trajectory.addSegment(coefficients, 1., mStartState);

  EXPECT_EQ(2, trajectory.getNumDerivatives());
}

TEST_F(SplineTest, getNumDerivatives_IsCubic_ReturnsThree)
{
  Eigen::Matrix<double, 2, 4> coefficients;
  coefficients << 0., 0., 0., 1.,
                  0., 0., 0., 2.;

  Spline trajectory(mStateSpace, 0.);
  trajectory.addSegment(coefficients, 1., mStartState);

  EXPECT_EQ(3, trajectory.getNumDerivatives());
}

TEST_F(SplineTest, getNumDerivatives_IsHomogeneous_ReturnsZero)
{
  Eigen::Matrix2d coefficients1, coefficients2;
  coefficients1 << 0., 1.,
                   0., 2.;
  coefficients2 << 0., 3.,
                   0., 4.;

  Spline trajectory(mStateSpace, 0.);
  trajectory.addSegment(coefficients1, 1., mStartState);
  trajectory.addSegment(coefficients2, 1.);

  EXPECT_EQ(1, trajectory.getNumDerivatives());
}

TEST_F(SplineTest, getNumDerivatives_IsHeterogeneous_ReturnsMax)
{
  Eigen::Matrix2d coefficients1;
  Eigen::Matrix<double, 2, 3> coefficients2;
  coefficients1 << 0., 1.,
                   0., 2.;
  coefficients2 << 0., 0., 3.,
                   0., 0., 4.;

  Spline trajectory(mStateSpace, 0.);
  trajectory.addSegment(coefficients1, 2., mStartState);
  trajectory.addSegment(coefficients2, 3.);

  EXPECT_EQ(2, trajectory.getNumDerivatives());
}

TEST_F(SplineTest, getStartTime)
{
  Spline trajectory(mStateSpace, 3.);
  EXPECT_DOUBLE_EQ(3., trajectory.getStartTime());
}

TEST_F(SplineTest, getEndTime_IsEmpty_ReturnsStartTime)
{
  Spline trajectory(mStateSpace, 3.);
  EXPECT_DOUBLE_EQ(3., trajectory.getEndTime());
}

TEST_F(SplineTest, getEndTime_IsNotEmpty_ReturnsEndTime)
{
  Spline trajectory(mStateSpace, 3.);

  trajectory.addSegment(Vector2d(4., 5.), 11., mStartState);
  EXPECT_DOUBLE_EQ(14., trajectory.getEndTime());

  trajectory.addSegment(Vector2d(6., 7.), 13.);
  EXPECT_DOUBLE_EQ(27., trajectory.getEndTime());
}

TEST_F(SplineTest, getDuration_IsEmpty_ReturnsZero)
{
  Spline trajectory(mStateSpace, 3.);
  EXPECT_DOUBLE_EQ(0., trajectory.getDuration());
}
  
TEST_F(SplineTest, getDuration_IsNotEmpty_ReturnsDuration)
{
  Spline trajectory(mStateSpace, 3.);

  trajectory.addSegment(Vector2d(4., 5.), 11., mStartState);
  EXPECT_DOUBLE_EQ(11., trajectory.getDuration());

  trajectory.addSegment(Vector2d(6., 7.), 13.);
  EXPECT_DOUBLE_EQ(24., trajectory.getDuration());
}

TEST_F(SplineTest, evaluate_IsEmpty_Throws)
{
  Spline trajectory(mStateSpace, 3.);

  auto state = mStateSpace->createState();

  EXPECT_THROW({
    trajectory.evaluate(3., state);
  }, std::logic_error);
}

TEST_F(SplineTest, evaluate_EvaluateStart_ReturnsStart)
{
  Matrix2d coefficients;
  coefficients << 0., 1.,
                  0., 2.;

  Spline trajectory(mStateSpace, 3.);
  trajectory.addSegment(coefficients, 1., mStartState);

  auto state = mStateSpace->createState();

  trajectory.evaluate(3., state);
  EXPECT_TRUE(START_VALUE.isApprox(state.getValue()));
}

TEST_F(SplineTest, evaluate_EvaluateEnd_ReturnsEnd)
{
  Matrix2d coefficients1, coefficients2;
  coefficients1 << 0., 1.,
                   0., 2.;
  coefficients2 << 0., 3.,
                   0., 4.;

  Spline trajectory(mStateSpace, 3.);
  auto state = mStateSpace->createState();

  trajectory.addSegment(coefficients1, 1., mStartState);
  trajectory.evaluate(4., state);
  EXPECT_TRUE(Vector2d(2., 4.).isApprox(state.getValue()));

  trajectory.addSegment(coefficients2, 2.);
  trajectory.evaluate(6., state);
  EXPECT_TRUE(Vector2d(8., 12.).isApprox(state.getValue()));
}

TEST_F(SplineTest, evaluate_Middle_ReturnsInterpolation)
{
  Eigen::Matrix<double, 2, 3> coefficients1, coefficients2, coefficients3;
  coefficients1 << 0., 0., 1.,
                   0., 1., 1.;
  coefficients2 << 0., 1., 2.,
                   0., 2., 2.;
  coefficients3 << 0., 2., 3.,
                   0., 3., 3.;

  Spline trajectory(mStateSpace, 3.);
  trajectory.addSegment(coefficients1, 1., mStartState);
  trajectory.addSegment(coefficients2, 2.);
  trajectory.addSegment(coefficients3, 3.);

  auto state = mStateSpace->createState();

  trajectory.evaluate(3.5, state);
  EXPECT_TRUE(Vector2d(1.25, 2.75).isApprox(state.getValue()));

  trajectory.evaluate(4.0, state);
  EXPECT_TRUE(Vector2d(2.00, 4.00).isApprox(state.getValue()));

  trajectory.evaluate(5.0, state);
  EXPECT_TRUE(Vector2d(5.00, 8.00).isApprox(state.getValue()));

  trajectory.evaluate(6.0, state);
  EXPECT_TRUE(Vector2d(12., 16.).isApprox(state.getValue()));

  trajectory.evaluate(7.5, state);
  EXPECT_TRUE(Vector2d(21.75, 27.25).isApprox(state.getValue()));

  trajectory.evaluate(9.0, state);
  EXPECT_TRUE(Vector2d(45.00, 52.00).isApprox(state.getValue()));
}

TEST_F(SplineTest, evaluateDerivative_IsEmpty_Throws)
{
  Spline trajectory(mStateSpace, 3.);
  Eigen::VectorXd tangentVector;

  EXPECT_THROW({
    trajectory.evaluateDerivative(3., 1, tangentVector);
  }, std::logic_error);
}

TEST_F(SplineTest, evaluateDerivative_FirstDerivative)
{
  Eigen::Matrix<double, 2, 3> coefficients1, coefficients2, coefficients3;
  coefficients1 << 0., 0., 1.,
                   0., 1., 1.;
  coefficients2 << 0., 1., 2.,
                   0., 2., 2.;
  coefficients3 << 0., 2., 3.,
                   0., 3., 3.;

  Spline trajectory(mStateSpace, 3.);
  trajectory.addSegment(coefficients1, 1., mStartState);
  trajectory.addSegment(coefficients2, 2.);
  trajectory.addSegment(coefficients3, 3.);
 
  Eigen::VectorXd tangentVector;

  trajectory.evaluateDerivative(3.5, 1, tangentVector);
  EXPECT_TRUE(Vector2d(1.00, 2.00).isApprox(tangentVector));

  trajectory.evaluateDerivative(5.0, 1, tangentVector);
  EXPECT_TRUE(Vector2d(5.00, 6.00).isApprox(tangentVector));

  trajectory.evaluateDerivative(7.5, 1, tangentVector);
  EXPECT_TRUE(Vector2d(11., 12.).isApprox(tangentVector));
}

TEST_F(SplineTest, EvaluateDerivative_SecondDerivative)
{
  Eigen::Matrix<double, 2, 3> coefficients1, coefficients2, coefficients3;
  coefficients1 << 0., 0., 1.,
                   0., 1., 1.;
  coefficients2 << 0., 1., 2.,
                   0., 2., 2.;
  coefficients3 << 0., 2., 3.,
                   0., 3., 3.;

  Spline trajectory(mStateSpace, 3.);
  trajectory.addSegment(coefficients1, 1., mStartState);
  trajectory.addSegment(coefficients2, 2.);
  trajectory.addSegment(coefficients3, 3.);
 
  Eigen::VectorXd tangentVector;

  trajectory.evaluateDerivative(3.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(2., 2.).isApprox(tangentVector));

  trajectory.evaluateDerivative(5.0, 2, tangentVector);
  EXPECT_TRUE(Vector2d(4., 4.).isApprox(tangentVector));

  trajectory.evaluateDerivative(7.5, 2, tangentVector);
  EXPECT_TRUE(Vector2d(6., 6.).isApprox(tangentVector));
}

TEST_F(SplineTest, EvaluateDerivative_HigherOrder_ReturnsZero)
{
  Eigen::Matrix<double, 2, 3> coefficients1, coefficients2, coefficients3;
  coefficients1 << 0., 0., 1.,
                   0., 1., 1.;
  coefficients2 << 0., 1., 2.,
                   0., 2., 2.;
  coefficients3 << 0., 2., 3.,
                   0., 3., 3.;

  Spline trajectory(mStateSpace, 3.);
  trajectory.addSegment(coefficients1, 1., mStartState);
  trajectory.addSegment(coefficients2, 2.);
  trajectory.addSegment(coefficients3, 3.);
 
  Eigen::VectorXd tangentVector;

  trajectory.evaluateDerivative(3.5, 3, tangentVector);
  EXPECT_TRUE(Vector2d::Zero().isApprox(tangentVector));

  trajectory.evaluateDerivative(5.0, 3, tangentVector);
  EXPECT_TRUE(Vector2d::Zero().isApprox(tangentVector));

  trajectory.evaluateDerivative(7.5, 3, tangentVector);
  EXPECT_TRUE(Vector2d::Zero().isApprox(tangentVector));
}
