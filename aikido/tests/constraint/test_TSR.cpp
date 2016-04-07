#include <aikido/constraint/TSR.hpp>
#include <aikido/state/State.hpp>
#include <aikido/util/RNG.hpp>
#include <dart/common/StlHelpers.h>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::TSR;
using aikido::constraint::TSRSampleGenerator;
using aikido::statespace::SE3StateSpace;
using aikido::util::RNGWrapper;
using aikido::util::RNG;
using dart::common::make_unique;

using DefaultRNG = RNGWrapper<std::default_random_engine>;

static std::unique_ptr<DefaultRNG> make_rng()
{
  return make_unique<RNGWrapper<std::default_random_engine>>(0);
}

TEST(TSR, InitializesToIdentity)
{
  TSR tsr;
  TSR tsr_default;

  EXPECT_EQ(tsr.mT0_w.translation(), 
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr.mT0_w.rotation(), 
            Eigen::Isometry3d::Identity().rotation());
  EXPECT_EQ(tsr.mBw, Eigen::MatrixXd::Zero(6,2));
  EXPECT_EQ(tsr.mTw_e.translation(),
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr.mTw_e.rotation(),
            Eigen::Isometry3d::Identity().rotation());

  EXPECT_EQ(tsr_default.mT0_w.translation(), 
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr_default.mT0_w.rotation(), 
            Eigen::Isometry3d::Identity().rotation());
  EXPECT_EQ(tsr_default.mBw, Eigen::MatrixXd::Zero(6,2));
  EXPECT_EQ(tsr_default.mTw_e.translation(),
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr_default.mTw_e.rotation(),
            Eigen::Isometry3d::Identity().rotation());
}

TEST(TSR, CopyConstructor)
{
  TSR tsr;
  TSR tsr2(tsr);

  EXPECT_EQ(tsr2.mT0_w.translation(), 
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr2.mT0_w.rotation(), 
            Eigen::Isometry3d::Identity().rotation());
  EXPECT_EQ(tsr2.mBw, Eigen::MatrixXd::Zero(6,2));
  EXPECT_EQ(tsr2.mTw_e.translation(), 
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr2.mTw_e.rotation(), 
            Eigen::Isometry3d::Identity().rotation());


  tsr2.mBw(0,0) = 3;
  EXPECT_THROW(tsr2.validate(), std::invalid_argument);
  EXPECT_NO_THROW(tsr.validate());

}


TEST(TSR, AssignmentOperator)
{
  TSR tsr;
  TSR tsr2 = tsr;

  EXPECT_EQ(tsr2.mT0_w.translation(), 
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr2.mT0_w.rotation(), 
            Eigen::Isometry3d::Identity().rotation());;
  EXPECT_EQ(tsr2.mBw, Eigen::MatrixXd::Zero(6,2));
  EXPECT_EQ(tsr2.mTw_e.translation(), 
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr2.mTw_e.rotation(), 
            Eigen::Isometry3d::Identity().rotation());

  tsr2.mBw(0,0) = 3;

  EXPECT_THROW(tsr2.validate(), std::invalid_argument);
  EXPECT_NO_THROW(tsr.validate());

}



TEST(TSR, Validate)
{
  TSR tsr; 

  tsr.mBw = Eigen::Matrix<double, 6, 2>::Zero();
  tsr.mBw(3,0) = M_PI/2;
  tsr.mBw(3,1) = -M_PI/2;
  
  EXPECT_THROW(tsr.validate(), std::invalid_argument);

  Eigen::MatrixXd Bw = Eigen::MatrixXd::Zero(6,2);
  Bw(3,0) = M_PI;
  Bw(3,1) = 2*M_PI;
  tsr.mBw = Bw;

  EXPECT_NO_THROW(tsr.validate());
}

TEST(TSRSampleGenerator, SampleIdentity)
{
  TSR tsr;
  auto state = tsr.getSE3StateSpace()->createState();

  ASSERT_TRUE(tsr.createSampleGenerator()->sample(state));
  ASSERT_TRUE(state.getIsometry().isApprox(Eigen::Isometry3d::Identity()));
}

TEST(TSRSampleGenerator, SamplePointTSR)
{
  TSR tsr;

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  Eigen::Isometry3d T0_w;
  T0_w.setIdentity();
  T0_w.linear() = rotation;
  tsr.mT0_w = T0_w;
  tsr.validate();

  auto state = tsr.getSE3StateSpace()->createState();
  ASSERT_TRUE(tsr.createSampleGenerator()->sample(state));
  EXPECT_TRUE(state.getIsometry().isApprox(T0_w));
}

TEST(TSRSampleGenerator, SampleWithinBounds)
{
  TSR tsr;

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0,0) = -1;
  Bw(0,1) = 1;

  tsr.mBw = Bw;

  auto sampler = tsr.createSampleGenerator();
  auto state = tsr.getSE3StateSpace()->createState();
  
  for (int i = 0; i < 10; i++)
  {
    ASSERT_TRUE(sampler->sample(state));

    Eigen::Vector3d translation = state.getIsometry().translation();
    EXPECT_TRUE(translation(0) >= -1 && translation(1) <= 1);
    EXPECT_DOUBLE_EQ(translation(1), 0);
    EXPECT_DOUBLE_EQ(translation(2), 0);
  }
}

TEST(TSRSampleGenerator, SampleSameSequence)
{
  TSR tsr;

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0,0) = -1;
  Bw(0,1) = 1;

  tsr.mBw = Bw;

  auto sampler1 = tsr.createSampleGenerator();
  auto sampler2 = tsr.createSampleGenerator();

  auto state1 = tsr.getSE3StateSpace()->createState();
  auto state2 = tsr.getSE3StateSpace()->createState();

  for (int i = 0; i < 10; i++)
  {
    ASSERT_TRUE(sampler1->sample(state1));
    ASSERT_TRUE(sampler2->sample(state2));
    EXPECT_TRUE(state1.getIsometry().isApprox(state2.getIsometry()));
  }
}
