#include <aikido/sampleable/TSR.hpp>
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::sampleable::TSR;
using aikido::util::RNGWrapper;
using aikido::util::RNG;


TEST(TSR, InitializesToIdentity)
{
  TSR tsr(std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));

  ASSERT_TRUE(tsr.mT0_w.isApprox(Eigen::Isometry3d::Identity()));
  ASSERT_TRUE(tsr.mBw.isApprox(Eigen::Matrix<double, 6, 2>::Zero()));
  ASSERT_TRUE(tsr.mTw_e.isApprox(Eigen::Isometry3d::Identity()));

  TSR tsr_default;

  ASSERT_TRUE(tsr_default.mT0_w.isApprox(Eigen::Isometry3d::Identity()));
  ASSERT_TRUE(tsr_default.mBw.isApprox(Eigen::Matrix<double, 6, 2>::Zero()));
  ASSERT_TRUE(tsr_default.mTw_e.isApprox(Eigen::Isometry3d::Identity()));

}

TEST(TSR, CopyConstructor)
{
  TSR tsr(std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));
  TSR tsr2(tsr);

  ASSERT_TRUE(tsr2.mT0_w.isApprox(Eigen::Isometry3d::Identity()));
  ASSERT_TRUE(tsr2.mBw.isApprox(Eigen::Matrix<double, 6, 2>::Zero()));
  ASSERT_TRUE(tsr2.mTw_e.isApprox(Eigen::Isometry3d::Identity()));

  tsr2.mBw(0,0) = 3;

  try
  {
    tsr2.validate();
    FAIL() << "Expected invalid argument error.";

  }
  catch(const std::invalid_argument& err)
  {
    EXPECT_EQ(err.what(),std::string("Lower bound must be less than upper bound."));
  }
  try
  {
    tsr.validate();
  }
  catch(...)
  {
    FAIL() << "Expected to pass.";
  }

}


TEST(TSR, AssignmentOperator)
{
  TSR tsr(std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));
  TSR tsr2 = tsr;

  ASSERT_TRUE(tsr.mT0_w.isApprox(Eigen::Isometry3d::Identity()));
  ASSERT_TRUE(tsr.mBw.isApprox(Eigen::Matrix<double, 6, 2>::Zero()));
  ASSERT_TRUE(tsr.mTw_e.isApprox(Eigen::Isometry3d::Identity()));

  ASSERT_TRUE(tsr2.mT0_w.isApprox(Eigen::Isometry3d::Identity()));
  ASSERT_TRUE(tsr2.mBw.isApprox(Eigen::Matrix<double, 6, 2>::Zero()));
  ASSERT_TRUE(tsr2.mTw_e.isApprox(Eigen::Isometry3d::Identity()));


  tsr2.mBw(0,0) = 3;

  try
  {
    tsr2.validate();
    FAIL() << "Expected invalid argument error.";

  }
  catch(const std::invalid_argument& err)
  {
    EXPECT_EQ(err.what(),std::string("Lower bound must be less than upper bound."));
  }
  try
  {
    tsr.validate();
  }
  catch(...)
  {
    FAIL() << "Expected to pass.";
  }

}


TEST(TSR, Wrap)
{
  TSR tsr;

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(3,0) = M_PI;
  Bw(3,1) = M_PI/2;

  tsr.mBw = Bw;
  tsr.wrap();
  ASSERT_DOUBLE_EQ(tsr.mBw(3,0), -M_PI);
  ASSERT_DOUBLE_EQ(tsr.mBw(3,1), M_PI/2);
}

TEST(TSR, Validate)
{
  TSR tsr; 
  tsr.mBw = Eigen::Matrix<double, 6, 2>::Zero();
  tsr.mBw(3,0) = M_PI/2;
  tsr.mBw(3,1) = -M_PI/2;
  try
  {
    tsr.validate();
    FAIL() << "Expected invalid argument error.";

  }
  catch(const std::invalid_argument& err)
  {
    EXPECT_EQ(err.what(),std::string("Lower bound must be less than upper bound."));
  }
  catch(...)
  {
    FAIL() << "Expected invalid argument error.";
  }

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(3,0) = 2*M_PI;
  Bw(3,1) = M_PI;
  tsr.mBw = Bw;
  try
  {
    tsr.wrap();
    tsr.validate();
  }
  catch(const std::invalid_argument& err)
  {
    EXPECT_EQ(err.what(),std::string("Lower bound must be less than upper bound."));
  }
  catch(...)
  {
    FAIL() << "Expected invalid argument error.";
  }

}


TEST(TSRSampleGenerator, SampleIdentity)
{
  TSR tsr(std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));

  ASSERT_TRUE(tsr.mT0_w.isApprox(Eigen::Isometry3d::Identity()));
  ASSERT_TRUE(tsr.mBw.isApprox(Eigen::Matrix<double, 6, 2>::Zero()));
  ASSERT_TRUE(tsr.mTw_e.isApprox(Eigen::Isometry3d::Identity()));

  ASSERT_TRUE(tsr.sampler()->sample().get().isApprox(Eigen::Isometry3d::Identity()));
}

TEST(TSRSampleGenerator, SamplePointTSR)
{
  TSR tsr(std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  Eigen::Isometry3d T0_w;
  T0_w.setIdentity();
  T0_w.linear() = rotation;
  tsr.mT0_w = T0_w;
  tsr.validate();

  ASSERT_TRUE(tsr.sampler()->sample().get().isApprox(T0_w));
}

TEST(TSRSampleGenerator, SampleWithinBounds)
{
  TSR tsr(std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0,0) = -1;
  Bw(0,1) = 1;

  tsr.mBw = Bw;
  tsr.wrap();
  tsr.validate();

  aikido::sampleable::TSRSamplerUniquePtr sampler(std::move(tsr.sampler()));
  for (int i = 0; i < 10; i++)
  {
    Eigen::Isometry3d sampleIsometry = sampler->sample().get();
    Eigen::Vector3d translation = sampleIsometry.translation();
    ASSERT_TRUE(translation(0) >= -1 && translation(1) <= 1);
    ASSERT_DOUBLE_EQ(translation(1), 0);
    ASSERT_DOUBLE_EQ(translation(2), 0);
  }

}
