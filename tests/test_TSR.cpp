#include <aikido/tsr/TSR.hpp>
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::tsr::TSR;
using aikido::util::RNGWrapper;

using BwMatrix = Eigen::Matrix<double, 6, 2>;

TEST(TSR, InitializesToIdentity)
{
  TSR tsr;

  ASSERT_TRUE(tsr.mT0_w.isApprox(Eigen::Isometry3d::Identity()));
  ASSERT_TRUE(tsr.mBw.isApprox(BwMatrix::Zero()));
  ASSERT_TRUE(tsr.mTw_e.isApprox(Eigen::Isometry3d::Identity()));

  RNGWrapper<std::default_random_engine> rng(0);
  ASSERT_TRUE(tsr.sample(rng).isApprox(Eigen::Isometry3d::Identity()));
}
