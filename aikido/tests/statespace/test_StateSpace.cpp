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

  EXPECT_TRUE(out.mValue.isApprox(Eigen::Vector3d(3,5,7)));

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
  SO3State identity;
  EXPECT_TRUE(identity.getIsometry().isApprox(Eigen::Isometry3d::Identity()));

  SO3State s2(Eigen::Vector3d(0, 0, M_PI/2));
  SO3State s3(Eigen::Vector3d(0, 0, M_PI/2));
  SO3State expected(Eigen::Vector3d(0, 0, M_PI));

  SO3State out;
  SO3StateSpace so3;
  so3.compose(s2, s3, out);

  EXPECT_TRUE(expected.getIsometry().isApprox(out.getIsometry()));
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

  std::vector<StateSpacePtr> ssp;
  ssp.push_back(std::make_shared<SO2StateSpace>());
  ssp.push_back(std::make_shared<RealVectorStateSpace>(2));

  CompoundStateSpace space(ssp);

  Eigen::Vector3d expected_q(M_PI/2,4,6);

  CompoundState cs1(Eigen::Vector3d(M_PI/4,1,2)),
                cs2(Eigen::Vector3d(M_PI/4,3,4)),
                out(3),
                expected(expected_q);

  space.compose(cs1, cs2, out);

  EXPECT_TRUE(out.mQ.isApprox(expected.mQ));

}

