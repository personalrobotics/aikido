#include <aikido/path/PiecewiseLinearTrajectory.hpp>
#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::path;
using namespace aikido::statespace;
using std::shared_ptr;
using std::make_shared;

class PiecewiseLinearTrajectoryTest : public ::testing::Test
{
public:
  void SetUp()
  {
    rvss = make_shared<RealVectorStateSpace>(2);

    auto s1 = rvss->createState();
    rvss->setValue(s1, Eigen::Vector2d(0, 0));
    auto s2 = rvss->createState();
    rvss->setValue(s2, Eigen::Vector2d(3, 3));
    auto s3 = rvss->createState();
    rvss->setValue(s3, Eigen::Vector2d(8, 1));

    dmetric = createDistanceMetricFor(rvss);
    traj = make_shared<PiecewiseLinearTrajectory>(rvss, dmetric);
    traj->addWaypoint(1, s1);
    traj->addWaypoint(3, s2);
    traj->addWaypoint(7, s3);
  }

  shared_ptr<RealVectorStateSpace> rvss;
  shared_ptr<DistanceMetric> dmetric;
  shared_ptr<PiecewiseLinearTrajectory> traj;
};

TEST_F(PiecewiseLinearTrajectoryTest, AddWaypoint)
{
  EXPECT_DOUBLE_EQ(1, traj->getFirstWaypointTime());
  EXPECT_DOUBLE_EQ(7, traj->getLastWaypointTime());
  EXPECT_DOUBLE_EQ(6, traj->getDuration());
  EXPECT_EQ(1, traj->getNumDerivatives());
}

TEST_F(PiecewiseLinearTrajectoryTest, EvaluatePt)
{
  using StateType = RealVectorStateSpace::State;

  EXPECT_THROW(traj->evaluate(-0.001), std::invalid_argument);
  EXPECT_THROW(traj->evaluate(9), std::invalid_argument);

  auto i1 = static_cast<StateType*>(traj->evaluate(1.5));
  EXPECT_TRUE(rvss->getValue(i1).isApprox(Eigen::Vector2d(.75, .75)));

  auto i2 = static_cast<StateType*>(traj->evaluate(6));
  EXPECT_TRUE(
      rvss->getValue(i2).isApprox(Eigen::Vector2d(3 + 5 * 0.75, 3 - 2 * .75)));

  auto i3 = static_cast<StateType*>(traj->evaluate(7));
  EXPECT_TRUE(rvss->getValue(i3).isApprox(Eigen::Vector2d(8, 1)));

  auto i4 = static_cast<StateType*>(traj->evaluate(3));
  EXPECT_TRUE(rvss->getValue(i4).isApprox(Eigen::Vector2d(3, 3)));
}

TEST_F(PiecewiseLinearTrajectoryTest, EvaluateDerivative)
{
  using StateType = RealVectorStateSpace::State;

  EXPECT_THROW(traj->evaluate(1.5, 0), std::invalid_argument);

  auto d1 = traj->evaluate(1.5, 2);
  EXPECT_TRUE(d1.isApprox(Eigen::Vector2d::Zero()));

  auto d2 = traj->evaluate(1.5, 1);
  EXPECT_TRUE(d2.isApprox(Eigen::Vector2d(1.5, 1.5)));

  auto d3 = traj->evaluate(6, 1);
  EXPECT_TRUE(d3.isApprox(Eigen::Vector2d(5. / 4, -2. / 4)));
}
