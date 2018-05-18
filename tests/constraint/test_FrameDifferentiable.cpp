#include <aikido/common/RNG.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/dart/FrameDifferentiable.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

#include <Eigen/Dense>
#include <gtest/gtest.h>

using aikido::constraint::dart::FrameDifferentiable;
using aikido::constraint::dart::TSR;
using aikido::statespace::SE3;
using aikido::statespace::SO2;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::common::RNG;
using aikido::common::RNGWrapper;
using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::FreeJoint;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;

class FrameDifferentiableTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {

    tsr = dart::common::make_aligned_shared<TSR>(
        std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));

    Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
    Bw(2, 0) = 1;
    Bw(2, 1) = 1;

    tsr->mBw = Bw;

    mSkeleton = Skeleton::create("manipulator");

    // Root joint
    FreeJoint::Properties jointProperties1;
    jointProperties1.mName = "root joint";

    BodyNode::Properties bodyProperties1;
    bodyProperties1.mName = "body1";

    bn1 = mSkeleton
              ->createJointAndBodyNodePair<FreeJoint>(
                  nullptr, jointProperties1, bodyProperties1)
              .second;

    // joint 2, body 2
    RevoluteJoint::Properties jointProperties2;
    jointProperties2.mAxis = Eigen::Vector3d::UnitY();
    jointProperties2.mName = "joint 2";
    jointProperties2.mT_ParentBodyToJoint.translation()
        = Eigen::Vector3d::UnitZ();

    BodyNode::Properties bodyProperties2;
    bodyProperties2.mName = "body2";

    bn2 = mSkeleton
              ->createJointAndBodyNodePair<RevoluteJoint>(
                  bn1, jointProperties2, bodyProperties2)
              .second;

    mSpace = std::make_shared<MetaSkeletonStateSpace>(mSkeleton.get());
  }

  BodyNodePtr bn1, bn2;
  SkeletonPtr mSkeleton;
  MetaSkeletonStateSpacePtr mSpace;
  std::shared_ptr<TSR> tsr;
};

TEST_F(FrameDifferentiableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(
      FrameDifferentiable(nullptr, nullptr, bn2.get(), tsr),
      std::invalid_argument);
}

TEST_F(FrameDifferentiableTest, ConstructorThrowsOnNullJacobianNode)
{
  EXPECT_THROW(
      FrameDifferentiable(mSpace, mSkeleton, nullptr, tsr),
      std::invalid_argument);
}

TEST_F(FrameDifferentiableTest, ConstructorThrowsOnNullPoseConstraint)
{
  EXPECT_THROW(
      FrameDifferentiable(mSpace, mSkeleton, bn2.get(), nullptr),
      std::invalid_argument);
}

TEST_F(FrameDifferentiableTest, ConstructorThrowsOnInvalidPoseConstraint)
{
  auto so2 = std::make_shared<SO2>();
  auto pconstraint = std::make_shared<aikido::constraint::Satisfied>(so2);
  EXPECT_THROW(
      FrameDifferentiable(mSpace, mSkeleton, bn2.get(), pconstraint),
      std::invalid_argument);
}

TEST_F(FrameDifferentiableTest, ConstraintDimension)
{
  FrameDifferentiable adaptor(mSpace, mSkeleton, bn2.get(), tsr);
  EXPECT_EQ(tsr->getConstraintDimension(), adaptor.getConstraintDimension());
}

TEST_F(FrameDifferentiableTest, StateSpaceMatch)
{
  FrameDifferentiable adaptor(mSpace, mSkeleton, bn2.get(), tsr);
  EXPECT_EQ(mSpace, adaptor.getStateSpace());
}

TEST_F(FrameDifferentiableTest, ConstraintTypes)
{
  FrameDifferentiable adaptor(mSpace, mSkeleton, bn2.get(), tsr);
  std::vector<aikido::constraint::ConstraintType> ctypes
      = adaptor.getConstraintTypes();
  std::vector<aikido::constraint::ConstraintType> tsrTypes
      = tsr->getConstraintTypes();
  EXPECT_EQ(tsrTypes.size(), ctypes.size());
  for (std::size_t i = 0; i < ctypes.size(); ++i)
    EXPECT_EQ(tsrTypes[i], ctypes[i]);
}

TEST_F(FrameDifferentiableTest, Value)
{
  FrameDifferentiable adaptor(mSpace, mSkeleton, bn2.get(), tsr);
  auto state = mSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());
  state.getSubStateHandle<SE3>(0).setIsometry(Eigen::Isometry3d::Identity());
  state.getSubStateHandle<SO2>(1).fromAngle(0);

  Eigen::VectorXd value;
  adaptor.getValue(state, value);

  EXPECT_TRUE(value.isApprox(Eigen::VectorXd::Zero(6)));

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;
  state.getSubStateHandle<SE3>(0).setIsometry(isometry);

  adaptor.getValue(state, value);

  Eigen::VectorXd expected(Eigen::VectorXd::Zero(6));
  expected(2) = 2;
  expected(3) = M_PI;
  expected(5) = M_PI;

  EXPECT_TRUE(value.isApprox(expected));
}

TEST_F(FrameDifferentiableTest, Jacobian)
{

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  for (int i = 0; i < 6; ++i)
  {
    Bw(i, 0) = -0.2;
    Bw(i, 1) = 0.2;
  }
  Bw(2, 0) = 0.8;
  Bw(2, 1) = 1.2;

  tsr->mBw = Bw;

  FrameDifferentiable adaptor(mSpace, mSkeleton, bn1.get(), tsr);

  // state strictly inside tsr
  auto state = mSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(0, 0, 1);
  state.getSubStateHandle<SE3>(0).setIsometry(isometry);
  state.getSubStateHandle<SO2>(1).fromAngle(0);

  Eigen::VectorXd value;
  adaptor.getValue(state, value);
  EXPECT_TRUE(value.isApprox(Eigen::VectorXd::Zero(6)));

  Eigen::MatrixXd jacobian;
  adaptor.getJacobian(state, jacobian);
  EXPECT_TRUE(jacobian.isApproxToConstant(0, 1e-3));

  // state outside tsr
  isometry.translation() = Eigen::Vector3d(0, 0, 2);
  state.getSubStateHandle<SE3>(0).setIsometry(isometry);

  adaptor.getJacobian(state, jacobian);

  Eigen::MatrixXd expected(Eigen::MatrixXd::Zero(6, 7));
  expected(2, 5) = 1;
  EXPECT_TRUE(jacobian.isApprox(expected, 1e-3));
}
