#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/SO3StateSpace.hpp>
#include <aikido/statespace/SE2StateSpace.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <dart/math/Geometry.h>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <typeinfo>       // std::bad_cast

using namespace aikido::statespace;
using namespace std;

TEST(RealVectorStateSpace, Compose)
{
  RealVectorStateSpace rvss(3);

  Eigen::Vector3d v(1,2,3);
  RealVectorStateSpace::State s1(Eigen::Vector3d(1, 2, 3));
  RealVectorStateSpace::State s2(Eigen::Vector3d(2, 3, 4));
  RealVectorStateSpace::State out(Eigen::Vector3d(Eigen::Vector3d::Zero()));

  rvss.compose(s1, s2, out);

  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector3d(3,5,7)));

}


TEST(SO2StateSpace, Compose)
{
  SO2StateSpace::State s1(M_PI/4);
  SO2StateSpace::State s2(M_PI/2);
  SO2StateSpace::State out;
  SO2StateSpace::State expected(3.0/4.0*M_PI);

  SO2StateSpace so2;
  so2.compose(s1, s2, out);

  EXPECT_TRUE(out.getRotation().isApprox(expected.getRotation()));

}


TEST(SO3StateSpace, Compose)
{
  SO3StateSpace::State identity;
  EXPECT_TRUE(identity.getQuaternion().isApprox(
    Eigen::Quaterniond::Identity()));

  SO3StateSpace::State s2(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3StateSpace::State s3(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3StateSpace::State expected(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));

  SO3StateSpace::State out;
  SO3StateSpace so3;
  so3.compose(s2, s3, out);

  EXPECT_TRUE(expected.getQuaternion().isApprox(out.getQuaternion()));
}



TEST(SE2StateSpace, Compose)
{
  SE2State identity;
  EXPECT_TRUE(identity.getIsometry().isApprox(Eigen::Isometry2d::Identity()));

  SE2State s2(Eigen::Vector3d(M_PI/2, 0, 0));
  SE2State s3(Eigen::Vector3d(M_PI/4, 0, 0));
  SE2State expected(Eigen::Vector3d(3.0/4.0*M_PI, 0, 0));

  SE2State out;
  SE2StateSpace se2;
  se2.compose(s2, s3, out);

  EXPECT_TRUE(expected.getIsometry().isApprox(out.getIsometry()));
}



TEST(SE3StateSpace, Compose)
{
  SE3State identity;
  EXPECT_TRUE(identity.getIsometry().isApprox(Eigen::Isometry3d::Identity()));

  SE3State s2, s3, expected;
  s2.mQ(0) = M_PI/2;
  s3.mQ(0) = M_PI/4;
  expected.mQ(0) = 3.0/4.0*M_PI;

  SE3State out;
  SE3StateSpace se3;
  se3.compose(s2, s3, out);

  EXPECT_TRUE(expected.getIsometry().isApprox(out.getIsometry()));

  SE3State s4, s5, expected2;
  s4.mQ.bottomRows(3) = Eigen::Vector3d(1,2,3);
  s5.mQ.bottomRows(3) = Eigen::Vector3d(4,5,6);
  expected2.mQ.bottomRows(3) = Eigen::Vector3d(5,7,9);


  SE3State out2;
  se3.compose(s4, s5, out2);

  EXPECT_TRUE(expected2.getIsometry().isApprox(out2.getIsometry()));

}



TEST(CompoundStateSpace, Compose)
{

  CompoundStateSpace space({
    std::make_shared<SO2StateSpace>(),
    std::make_shared<RealVectorStateSpace>(2)
  });

  Eigen::Vector3d expected_q(M_PI/2,4,6);

  CompoundStateSpace::State cs1({
    new SO2StateSpace::State(M_PI_2),
    new RealVectorStateSpace::State(Eigen::Vector2d(3., 4.))
  });
  CompoundStateSpace::State cs2({
    new SO2StateSpace::State(M_PI_2),
    new RealVectorStateSpace::State(Eigen::Vector2d(5., 10.))
  });
  CompoundStateSpace::State out({
    new SO2StateSpace::State,
    new RealVectorStateSpace::State(Eigen::Vector2d::Zero())
  });

  space.compose(cs1, cs2, out);

  const auto& out1 = dynamic_cast<const SO2StateSpace::State&>(
    *out.getState(0));
  EXPECT_DOUBLE_EQ(M_PI, out1.getAngle());

  const auto& out2 = dynamic_cast<const RealVectorStateSpace::State&>(
    *out.getState(1));
  EXPECT_TRUE(out2.getValue().isApprox(Eigen::Vector2d(8., 14.)));
}
