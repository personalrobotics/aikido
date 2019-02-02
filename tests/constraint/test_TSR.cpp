#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <dart/common/StlHelpers.hpp>
#include <dart/math/Geometry.hpp>
#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/constraint/Differentiable.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/statespace/SE3.hpp>

using aikido::constraint::dart::TSR;
using aikido::constraint::ConstraintType;
using aikido::statespace::SE3;
using aikido::common::RNGWrapper;
using aikido::common::RNG;

using DefaultRNG = RNGWrapper<std::default_random_engine>;

TEST(TSR, InitializesToIdentity)
{
  TSR tsr;
  TSR tsr_default;

  EXPECT_EQ(
      tsr.mT0_w.translation(), Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr.mT0_w.rotation(), Eigen::Isometry3d::Identity().rotation());
  EXPECT_EQ(tsr.mBw, Eigen::MatrixXd::Zero(6, 2));
  EXPECT_EQ(
      tsr.mTw_e.translation(), Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr.mTw_e.rotation(), Eigen::Isometry3d::Identity().rotation());

  EXPECT_EQ(
      tsr_default.mT0_w.translation(),
      Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(
      tsr_default.mT0_w.rotation(), Eigen::Isometry3d::Identity().rotation());
  EXPECT_EQ(tsr_default.mBw, Eigen::MatrixXd::Zero(6, 2));
  EXPECT_EQ(
      tsr_default.mTw_e.translation(),
      Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(
      tsr_default.mTw_e.rotation(), Eigen::Isometry3d::Identity().rotation());
}

TEST(TSR, CopyConstructor)
{
  TSR tsr;
  TSR tsr2(tsr);

  EXPECT_EQ(
      tsr2.mT0_w.translation(), Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr2.mT0_w.rotation(), Eigen::Isometry3d::Identity().rotation());
  EXPECT_EQ(tsr2.mBw, Eigen::MatrixXd::Zero(6, 2));
  EXPECT_EQ(
      tsr2.mTw_e.translation(), Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr2.mTw_e.rotation(), Eigen::Isometry3d::Identity().rotation());

  tsr2.mBw(0, 0) = 3;
  EXPECT_THROW(tsr2.validate(), std::logic_error);
  EXPECT_NO_THROW(tsr.validate());
}

TEST(TSR, AssignmentOperator)
{
  TSR tsr;
  TSR tsr2 = tsr;

  EXPECT_EQ(
      tsr2.mT0_w.translation(), Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr2.mT0_w.rotation(), Eigen::Isometry3d::Identity().rotation());
  ;
  EXPECT_EQ(tsr2.mBw, Eigen::MatrixXd::Zero(6, 2));
  EXPECT_EQ(
      tsr2.mTw_e.translation(), Eigen::Isometry3d::Identity().translation());
  EXPECT_EQ(tsr2.mTw_e.rotation(), Eigen::Isometry3d::Identity().rotation());

  tsr2.mBw(0, 0) = 3;

  EXPECT_THROW(tsr2.validate(), std::logic_error);
  EXPECT_NO_THROW(tsr.validate());
}

TEST(TSR, Validate)
{
  TSR tsr;

  tsr.mBw = Eigen::Matrix<double, 6, 2>::Zero();
  tsr.mBw(3, 0) = M_PI / 2;
  tsr.mBw(3, 1) = -M_PI / 2;

  EXPECT_THROW(tsr.validate(), std::logic_error);

  Eigen::MatrixXd Bw = Eigen::MatrixXd::Zero(6, 2);
  Bw(3, 0) = M_PI;
  Bw(3, 1) = 2 * M_PI;
  tsr.mBw = Bw;

  EXPECT_NO_THROW(tsr.validate());
}

TEST(TSRSampleGenerator, SampleIdentity)
{
  TSR tsr;
  auto state = tsr.getSE3()->createState();

  ASSERT_TRUE(tsr.createSampleGenerator()->sample(state));
  ASSERT_TRUE(state.getIsometry().isApprox(Eigen::Isometry3d::Identity()));
}

TEST(TSRSampleGenerator, SamplePointTSR)
{
  TSR tsr;

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  Eigen::Isometry3d T0_w;
  T0_w.setIdentity();
  T0_w.linear() = rotation;
  tsr.mT0_w = T0_w;
  tsr.validate();

  auto state = tsr.getSE3()->createState();
  auto generator = tsr.createSampleGenerator();

  ASSERT_TRUE(generator->canSample());
  ASSERT_EQ(1, generator->getNumSamples());
  ASSERT_TRUE(generator->sample(state));
  EXPECT_TRUE(state.getIsometry().isApprox(T0_w));

  ASSERT_FALSE(generator->canSample());
  ASSERT_EQ(0, generator->getNumSamples());
}

TEST(TSRSampleGenerator, SampleWithinBounds)
{
  TSR tsr;

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0, 0) = -1;
  Bw(0, 1) = 1;

  tsr.mBw = Bw;

  auto sampler = tsr.createSampleGenerator();
  auto state = tsr.getSE3()->createState();

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
  Bw(0, 0) = -1;
  Bw(0, 1) = 1;

  tsr.mBw = Bw;

  auto sampler1 = tsr.createSampleGenerator();
  auto sampler2 = tsr.createSampleGenerator();

  auto state1 = tsr.getSE3()->createState();
  auto state2 = tsr.getSE3()->createState();

  for (int i = 0; i < 10; i++)
  {
    ASSERT_TRUE(sampler1->sample(state1));
    ASSERT_TRUE(sampler2->sample(state2));
    EXPECT_TRUE(state1.getIsometry().isApprox(state2.getIsometry()));
  }
}

TEST(TSR, GetValue)
{
  TSR tsr;

  // non-trivial translation bounds
  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0, 0) = 0;
  Bw(0, 1) = 2;

  tsr.mBw = Bw;

  auto state = tsr.getSE3()->createState();

  // strictly inside TSR
  Eigen::Isometry3d isometry(Eigen::Isometry3d::Identity());
  isometry.translation() = Eigen::Vector3d(1.5, 0, 0);
  state.setIsometry(isometry);
  Eigen::VectorXd value;
  tsr.getValue(state, value);
  EXPECT_TRUE(value.isApproxToConstant(0, 1e-3));

  // strictly inside TSR
  isometry.translation() = Eigen::Vector3d(1, 0, 0);
  state.setIsometry(isometry);
  tsr.getValue(state, value);
  EXPECT_TRUE(value.isApproxToConstant(0));

  // boundary of TSR
  isometry.translation() = Eigen::Vector3d(2, 0, 0);
  state.setIsometry(isometry);
  tsr.getValue(state, value);
  EXPECT_TRUE(value.isApproxToConstant(0));

  // outside TSR
  isometry.translation() = Eigen::Vector3d(3, 0, 0);
  state.setIsometry(isometry);

  Eigen::Vector6d expected(Eigen::Vector6d::Zero());
  expected(0) = 1;
  tsr.getValue(state, value);
  EXPECT_TRUE(value.isApprox(expected));

  // TSR non-trivial angle bounds
  Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(3, 0) = 0;
  Bw(3, 1) = M_PI;

  tsr.mBw = Bw;

  // strictly inside TSR
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());

  isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = rotation;
  state.setIsometry(isometry);
  tsr.getValue(state, value);

  EXPECT_TRUE(value.isApproxToConstant(0, 1e-3));

  // [PI/2, PI/2+2PI-0.1]
  Bw(3, 0) = M_PI_2;
  Bw(3, 1) = M_PI_2 + M_PI * 2 - 0.1;
  tsr.mBw = Bw;

  rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;
  state.setIsometry(isometry);
  tsr.getValue(state, value);
  expected = Eigen::Vector6d::Zero();
  EXPECT_TRUE(value.isApproxToConstant(0, 1e-3));

  /* cyclic bound */
  Bw(3, 0) = M_PI_2;
  Bw(3, 1) = M_PI_2 + M_PI * 2;
  tsr.mBw = Bw;

  rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;
  state.setIsometry(isometry);
  tsr.getValue(state, value);
  expected = Eigen::Vector6d::Zero();
  EXPECT_TRUE(value.isApproxToConstant(0, 1e-3));

  // boundary of TSR
  rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  isometry.linear() = rotation;
  state.setIsometry(isometry);
  tsr.getValue(state, value);
  EXPECT_TRUE(value.isApproxToConstant(0));

  /* outside TSR */
  // [PI/2, PI] bound
  Bw(3, 0) = M_PI_2;
  Bw(3, 1) = M_PI;
  tsr.mBw = Bw;

  rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(M_PI_2 - 0.1, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;
  state.setIsometry(isometry);
  tsr.getValue(state, value);
  expected = Eigen::Vector6d::Zero();
  expected(3) = 0.1;
  EXPECT_TRUE(value.isApprox(expected));

  // Same bound, but with 5*2*pi added to each end.
  Bw(3, 0) = M_PI_2 + 5 * (M_PI * 2);
  Bw(3, 1) = M_PI + 5 * (M_PI * 2);

  tsr.mBw = Bw;

  rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(M_PI_2 - 0.1, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;
  state.setIsometry(isometry);
  tsr.getValue(state, value);
  expected = Eigen::Vector6d::Zero();
  expected(3) = 0.1;
  EXPECT_TRUE(value.isApprox(expected));

  // [PI/2, 2*PI]
  Bw(3, 0) = M_PI_2;
  Bw(3, 1) = M_PI * 2;
  tsr.mBw = Bw;

  rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(M_PI_4 - 0.1, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;
  state.setIsometry(isometry);
  tsr.getValue(state, value);
  expected = Eigen::Vector6d::Zero();
  expected(3) = M_PI_4 - 0.1;
  EXPECT_TRUE(value.isApprox(expected, 1e-3));
}

TEST(TSR, GetJacobian)
{
  TSR tsr;

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0, 0) = 0;
  Bw(0, 1) = 2;

  tsr.mBw = Bw;

  auto state = tsr.getSE3()->createState();

  // strictly inside TSR
  Eigen::Isometry3d isometry(Eigen::Isometry3d::Identity());
  isometry.translation() = Eigen::Vector3d(1.5, 0, 0);
  state.setIsometry(isometry);

  Eigen::MatrixXd jacobian;
  tsr.getJacobian(state, jacobian);
  EXPECT_TRUE(jacobian.isApproxToConstant(0));

  // outside TSR
  isometry.translation() = Eigen::Vector3d(3, 0, 0);
  state.setIsometry(isometry);
  tsr.getJacobian(state, jacobian);

  Eigen::Matrix6d expected(Eigen::Matrix6d::Zero());
  expected(0, 3) = 1;
  EXPECT_TRUE(jacobian.isApprox(expected, 1e-3));
}

TEST(TSR, GetValueAndJacobian)
{
  TSR tsr;

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0, 0) = 0;
  Bw(0, 1) = 2;

  tsr.mBw = Bw;

  auto state = tsr.getSE3()->createState();

  // strictly inside TSR
  Eigen::Isometry3d isometry(Eigen::Isometry3d::Identity());
  isometry.translation() = Eigen::Vector3d(1.5, 0, 0);
  state.setIsometry(isometry);

  Eigen::VectorXd valExpected, val;
  Eigen::MatrixXd jacExpected, jac;
  std::pair<Eigen::VectorXd, Eigen::MatrixXd> valueAndJacobian;
  tsr.getValue(state, valExpected);
  tsr.getJacobian(state, jacExpected);
  tsr.getValueAndJacobian(state, val, jac);

  EXPECT_TRUE(valExpected.isApprox(val));
  EXPECT_TRUE(jacExpected.isApprox(jac));
}

TEST(TSR, GetConstraintTypes)
{
  // This tests current behavior, but it may fail.
  TSR tsr;
  auto types = tsr.getConstraintTypes();

  EXPECT_EQ(6, types.size());

  for (auto t : types)
  {
    EXPECT_EQ(t, ConstraintType::INEQUALITY);
  }
}

TEST(TSR, GetConstraintDimension)
{
  TSR tsr;
  auto dim = tsr.getConstraintDimension();

  EXPECT_EQ(6, dim);
}

TEST(TSR, IsSatisfied)
{
  TSR tsr;

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0, 0) = 0;
  Bw(0, 1) = 2;

  tsr.mBw = Bw;

  auto state = tsr.getSE3()->createState();

  // strictly inside TSR
  Eigen::Isometry3d isometry(Eigen::Isometry3d::Identity());
  isometry.translation() = Eigen::Vector3d(1.5, 0, 0);
  state.setIsometry(isometry);

  EXPECT_TRUE(tsr.isSatisfied(state));

  // outside TSR
  isometry.translation() = Eigen::Vector3d(3, 0, 0);
  state.setIsometry(isometry);

  EXPECT_FALSE(tsr.isSatisfied(state));
}

TEST(TSR, getSE3EqualToGetStateSpace)
{
  TSR tsr;
  auto se3space = tsr.getSE3();
  auto space = tsr.getStateSpace();

  EXPECT_EQ(se3space, space);
}
