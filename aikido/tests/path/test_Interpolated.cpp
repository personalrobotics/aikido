#include <aikido/path/Interpolated.hpp>
#include <aikido/statespace/Rn.hpp>
#include <gtest/gtest.h>

using namespace aikido::path;
using namespace aikido::statespace;
using std::shared_ptr;
using std::make_shared;

class InterpolatedTest : public ::testing::Test
{
public:
  void SetUp()
  {
    rvss = make_shared<Rn>(2);
    interpolator = make_shared<GeodesicInterpolator>(rvss);

    auto s1 = rvss->createState();
    rvss->setValue(s1, Eigen::Vector2d(0, 0));
    auto s2 = rvss->createState();
    rvss->setValue(s2, Eigen::Vector2d(3, 3));
    auto s3 = rvss->createState();
    rvss->setValue(s3, Eigen::Vector2d(8, 1));

    traj = make_shared<Interpolated>(rvss, interpolator);
    traj->addWaypoint(1, s1);
    traj->addWaypoint(3, s2);
    traj->addWaypoint(7, s3);
  }

  shared_ptr<Rn> rvss;
  shared_ptr<Interpolator> interpolator;
  shared_ptr<Interpolated> traj;
};

TEST_F(InterpolatedTest, AddWaypoint)
{
  EXPECT_DOUBLE_EQ(1, traj->getStartTime());
  EXPECT_DOUBLE_EQ(7, traj->getEndTime());
  EXPECT_DOUBLE_EQ(6, traj->getDuration());
  EXPECT_EQ(1, traj->getNumDerivatives());
}

TEST_F(InterpolatedTest, EvaluatePt)
{
  using StateType = Rn::State;
  auto istate = rvss->createState();
  
  // Point before first time on traj
  traj->evaluate(-0.001, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(0, 0)));

  // Point after last time on traj
  traj->evaluate(8, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(8, 1)));
  
  traj->evaluate(1.5, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(.75, .75)));

  traj->evaluate(6, istate);
  EXPECT_TRUE(
      rvss->getValue(istate).isApprox(Eigen::Vector2d(3 + 5 * 0.75, 3 - 2 * .75)));

  traj->evaluate(7, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(8, 1)));

  traj->evaluate(3, istate);
  EXPECT_TRUE(rvss->getValue(istate).isApprox(Eigen::Vector2d(3, 3)));
}

TEST_F(InterpolatedTest, EvaluateDerivative)
{
  using StateType = Rn::State;

  EXPECT_THROW(traj->evaluate(1.5, 0), std::invalid_argument);

  auto d1 = traj->evaluate(1.5, 2);
  EXPECT_TRUE(d1.isApprox(Eigen::Vector2d::Zero()));

  auto d2 = traj->evaluate(1.5, 1);
  EXPECT_TRUE(d2.isApprox(Eigen::Vector2d(1.5, 1.5)));

  auto d3 = traj->evaluate(6, 1);
  EXPECT_TRUE(d3.isApprox(Eigen::Vector2d(5. / 4, -2. / 4)));
}
