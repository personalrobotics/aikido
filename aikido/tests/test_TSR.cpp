#include <aikido/constraint/TSR.hpp>
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::TSR;
using aikido::constraint::TSRSampleGenerator;

using aikido::util::RNGWrapper;
using aikido::util::RNG;


TEST(TSR, InitializesToIdentity)
{
  TSR tsr(std::unique_ptr<RNG>(
    new RNGWrapper<std::default_random_engine>(0)));

  EXPECT_EQ(tsr.mT0_w.translation(), 
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr.mT0_w.rotation(), 
            Eigen::Isometry3d::Identity().rotation());
  EXPECT_EQ(tsr.mBw, Eigen::MatrixXd::Zero(6,2));
  EXPECT_EQ(tsr.mTw_e.translation(),
            Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr.mTw_e.rotation(),
            Eigen::Isometry3d::Identity().rotation());

  TSR tsr_default;

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
  TSR tsr(std::unique_ptr<RNG>(
    new RNGWrapper<std::default_random_engine>(0)));
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
  TSR tsr(std::unique_ptr<RNG>(
    new RNGWrapper<std::default_random_engine>(0)));
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
  TSR tsr(std::unique_ptr<RNG>(
    new RNGWrapper<std::default_random_engine>(0)));

  boost::optional<Eigen::Isometry3d> sample = tsr.createSampleGenerator()->sample();

  ASSERT_TRUE(sample);
  ASSERT_TRUE(sample->isApprox(Eigen::Isometry3d::Identity()));
}


TEST(TSRSampleGenerator, SamplePointTSR)
{
  TSR tsr(std::unique_ptr<RNG>(
    new RNGWrapper<std::default_random_engine>(0)));
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  Eigen::Isometry3d T0_w;
  T0_w.setIdentity();
  T0_w.linear() = rotation;
  tsr.mT0_w = T0_w;
  tsr.validate();

  boost::optional<Eigen::Isometry3d> sample = tsr.createSampleGenerator()->sample();

  ASSERT_TRUE(sample);
  EXPECT_TRUE(sample->isApprox(T0_w));
}


TEST(TSRSampleGenerator, SampleWithinBounds)
{
  TSR tsr(std::unique_ptr<RNG>(
    new RNGWrapper<std::default_random_engine>(0)));

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0,0) = -1;
  Bw(0,1) = 1;

  tsr.mBw = Bw;

  aikido::constraint::TSRSamplerUniquePtr sampler(
    std::move(tsr.createSampleGenerator()));
  
  for (int i = 0; i < 10; i++)
  {
    boost::optional<Eigen::Isometry3d> sample = sampler->sample();
    ASSERT_TRUE(sample);
    Eigen::Vector3d translation = sample->translation();
    EXPECT_TRUE(translation(0) >= -1 && translation(1) <= 1);
    EXPECT_DOUBLE_EQ(translation(1), 0);
    EXPECT_DOUBLE_EQ(translation(2), 0);
  }

}


TEST(TSRSampleGenerator, SampleSameSequence)
{
  TSR tsr;(std::unique_ptr<RNG>(
    new RNGWrapper<std::default_random_engine>(0)));

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0,0) = -1;
  Bw(0,1) = 1;

  tsr.mBw = Bw;

  aikido::constraint::TSRSamplerUniquePtr sampler1(
    std::move(tsr.createSampleGenerator()));

  aikido::constraint::TSRSamplerUniquePtr sampler2(
    std::move(tsr.createSampleGenerator()));

  for (int i = 0; i < 10; i++)
  {
    boost::optional<Eigen::Isometry3d> sample1 = sampler1->sample();
    ASSERT_TRUE(sample1);
    boost::optional<Eigen::Isometry3d> sample2 = sampler2->sample();
    ASSERT_TRUE(sample2);
    EXPECT_TRUE(sample1.get().isApprox(sample2.get()));
  }

}

