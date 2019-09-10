#include <gtest/gtest.h>

#include <aikido/statespace/Rn.hpp>
#include <aikido/trajectory/BSpline.hpp>

using namespace aikido::statespace;
using aikido::trajectory::BSpline;
using Eigen::Matrix2d;
using Eigen::Vector2d;

static const Vector2d START_VALUE(1., 2.);

class BSplineTest : public ::testing::Test
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
  std::shared_ptr<BSpline> mTrajectory;
};

TEST_F(BSplineTest, addSegment_NegativeDurationThrows)
{
  EXPECT_THROW(BSpline(mStateSpace, 2, 3, 1.0, 0.5), std::invalid_argument);
}

TEST_F(BSplineTest, addSegment_ZeroDurationThrows)
{
  EXPECT_THROW(BSpline(mStateSpace, 2, 3, 0.5, 0.5), std::invalid_argument);
}

TEST_F(BSplineTest, addSegment_IncorrectDimension_Throws)
{
  Eigen::Vector3d vec(1.0, 1.0, 1.0);

  BSpline trajectory(mStateSpace, 2, 3);

  EXPECT_THROW({ trajectory.setStartPoint(vec); }, std::invalid_argument);
}

TEST_F(BSplineTest, getStateSpace)
{
  BSpline trajectory(mStateSpace, 2, 3);

  EXPECT_EQ(mStateSpace, trajectory.getStateSpace());
}

TEST_F(BSplineTest, getNumDerivatives_IsEmpty_ReturnsZero)
{
  BSpline trajectory(mStateSpace, 0, 3);

  EXPECT_EQ(0u, trajectory.getNumDerivatives());
}

TEST_F(BSplineTest, getStartTime)
{
  BSpline trajectory(mStateSpace, 2, 3, 3.0, 4.0);
  EXPECT_DOUBLE_EQ(3.0, trajectory.getStartTime());
}

TEST_F(BSplineTest, getEndTime_IsNotEmpty_ReturnsEndTime)
{
  BSpline trajectory(mStateSpace, 2, 3, 3.0, 7.0);
  EXPECT_DOUBLE_EQ(7., trajectory.getEndTime());
}

TEST_F(BSplineTest, getDuration_IsNotEmpty_ReturnsDuration)
{
  BSpline trajectory(mStateSpace, 2, 3, 3., 7.);
  EXPECT_DOUBLE_EQ(4., trajectory.getDuration());
}

TEST_F(BSplineTest, evaluate_OutOfDuration_Throws)
{
  BSpline trajectory(mStateSpace, 2, 3);

  auto state = mStateSpace->createState();
  EXPECT_THROW({ trajectory.evaluate(3., state); }, std::invalid_argument);
}

TEST_F(BSplineTest, evaluate_EvaluateStart_ReturnsStart)
{
  Eigen::Vector2d start(1., 1.);
  BSpline trajectory(mStateSpace, 2, 3);
  trajectory.setStartPoint(start);

  auto state = mStateSpace->createState();

  trajectory.evaluate(0., state);
  EXPECT_TRUE(start.isApprox(state.getValue()));
}

TEST_F(BSplineTest, evaluate_EvaluateEnd_ReturnsEnd)
{
  Eigen::Vector2d start(1.0, 1.0);
  Eigen::Vector2d end(2.0, 2.0);

  BSpline trajectory(mStateSpace, 2, 3);
  trajectory.setStartPoint(start);
  trajectory.setEndPoint(end);

  auto state = mStateSpace->createState();

  trajectory.evaluate(0.0, state);
  EXPECT_TRUE(start.isApprox(state.getValue()));

  trajectory.evaluate(1.0, state);
  EXPECT_TRUE(end.isApprox(state.getValue()));
}
