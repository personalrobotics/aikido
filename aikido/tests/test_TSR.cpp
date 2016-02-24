#include <aikido/sampleable/TSR.hpp>
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::sampleable::TSR;
using aikido::util::RNGWrapper;
using aikido::util::RNG;

using BwMatrix = Eigen::Matrix<double, 6, 2>;

TEST(TSR, InitializesToIdentity)
{
  RNGWrapper<std::default_random_engine> rng(0);
  TSR tsr(std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));

  ASSERT_TRUE(tsr.mT0_w.isApprox(Eigen::Isometry3d::Identity()));
  ASSERT_TRUE(tsr.mBw.isApprox(BwMatrix::Zero()));
  ASSERT_TRUE(tsr.mTw_e.isApprox(Eigen::Isometry3d::Identity()));

  ASSERT_TRUE(tsr.sampler()->sample().get().isApprox(Eigen::Isometry3d::Identity()));

}
