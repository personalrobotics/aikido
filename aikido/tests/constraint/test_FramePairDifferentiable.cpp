#include <aikido/constraint/FramePairDifferentiable.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/util/RNG.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::FramePairDifferentiable;
using aikido::constraint::TSR;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::SkeletonPtr;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using dart::dynamics::Skeleton;
using aikido::statespace::SE3;
using aikido::statespace::SO2;
using aikido::util::RNG;
using aikido::util::RNGWrapper;
using dart::dynamics::FreeJoint;
using dart::dynamics::BodyNode;

class FramePairDifferentiableTest : public ::testing::Test {
  protected:
    virtual void SetUp() {

      tsr = std::make_shared<TSR>(
        std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));

      Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
      tsr->mBw = Bw;

      skeleton = Skeleton::create("env");

      // body 1
      FreeJoint::Properties properties1;
      properties1.mName = "joint1";
      bn1 = skeleton->createJointAndBodyNodePair<FreeJoint>(
                        nullptr, properties1, 
                        BodyNode::Properties(std::string("body1"))).second;

      //   body 2
      FreeJoint::Properties properties2;
      properties2.mName = "joint 2";
      bn2 = skeleton->createJointAndBodyNodePair<FreeJoint>(
                        nullptr, properties2, 
                        BodyNode::Properties(std::string("body2"))).second;

      MetaSkeletonStateSpace space(skeleton);
      spacePtr = std::make_shared<MetaSkeletonStateSpace>(space);
  }

  BodyNodePtr bn1, bn2;
  SkeletonPtr skeleton;
  MetaSkeletonStateSpacePtr spacePtr;
  std::shared_ptr<TSR> tsr ;

};

TEST_F(FramePairDifferentiableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(FramePairDifferentiable(nullptr, bn1.get(), bn2.get(), tsr),
               std::invalid_argument);
}

TEST_F(FramePairDifferentiableTest, ConstructorThrowsOnNullNode1)
{
  EXPECT_THROW(FramePairDifferentiable(spacePtr, nullptr, bn2.get(), tsr),
               std::invalid_argument);
}

TEST_F(FramePairDifferentiableTest, ConstructorThrowsOnNullNode2)
{
  EXPECT_THROW(FramePairDifferentiable(spacePtr, bn1.get(), nullptr, tsr),
               std::invalid_argument);
}

TEST_F(FramePairDifferentiableTest, ConstructorThrowsOnNullPoseConstraint)
{
  EXPECT_THROW(FramePairDifferentiable(spacePtr, bn1.get(), bn2.get(), nullptr),
               std::invalid_argument);
}

TEST_F(FramePairDifferentiableTest, ConstructorThrowsOnInvalidPoseConstraint)
{
  auto so2 = std::make_shared<SO2>();
  auto pconstraint = std::make_shared<aikido::constraint::Satisfied>(so2);
  EXPECT_THROW(FramePairDifferentiable(spacePtr, bn1.get(), bn2.get(), pconstraint),
               std::invalid_argument);
}

TEST_F(FramePairDifferentiableTest, ConstraintDimension)
{
  FramePairDifferentiable adaptor(spacePtr, bn1.get(), bn2.get(), tsr);         
  EXPECT_EQ(tsr->getConstraintDimension(),
            adaptor.getConstraintDimension());
}

TEST_F(FramePairDifferentiableTest, StateSpaceMatch)
{
  FramePairDifferentiable adaptor(spacePtr, bn1.get(), bn2.get(), tsr);
  EXPECT_EQ(spacePtr, adaptor.getStateSpace());
}

TEST_F(FramePairDifferentiableTest, ConstraintTypes)
{
  FramePairDifferentiable adaptor(spacePtr, bn1.get(), bn2.get(), tsr);
  std::vector<aikido::constraint::ConstraintType> ctypes =
      adaptor.getConstraintTypes();
  std::vector<aikido::constraint::ConstraintType> tsrTypes =
      tsr->getConstraintTypes();
  EXPECT_EQ(tsrTypes.size(), ctypes.size());
  for(size_t i = 0; i < ctypes.size(); ++i)
      EXPECT_EQ(tsrTypes[i], ctypes[i]);
}

TEST_F(FramePairDifferentiableTest, Value)
{
  FramePairDifferentiable adaptor(spacePtr, bn1.get(), bn2.get(), tsr);
  auto state = spacePtr->getScopedStateFromMetaSkeleton();
  state.getSubStateHandle<SE3>(0).setIsometry(
    Eigen::Isometry3d::Identity());
  state.getSubStateHandle<SE3>(1).setIsometry(
    Eigen::Isometry3d::Identity());
  
  Eigen::VectorXd value;
  adaptor.getValue(state, value);
  EXPECT_TRUE(value.isApprox(Eigen::VectorXd::Zero(6)));

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(2,0) = 1;
  Bw(2,1) = 1;

  tsr->mBw = Bw;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  adaptor.getValue(state, value);
  
  Eigen::VectorXd expected(Eigen::VectorXd::Zero(6));
  expected(2) = 1;
  EXPECT_TRUE(value.isApprox(expected));
}

TEST_F(FramePairDifferentiableTest, Jacobian)
{

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  for(int i = 0; i < 6; ++i)
  {
    Bw(i, 0) = -0.2;
    Bw(i, 1) = 0.2;
  }

  tsr->mBw = Bw;

  FramePairDifferentiable adaptor(spacePtr, bn1.get(), bn2.get(), tsr);

  // state strictly inside tsr
  auto state = spacePtr->getScopedStateFromMetaSkeleton();
  state.getSubStateHandle<SE3>(0).setIsometry(
    Eigen::Isometry3d::Identity());
  state.getSubStateHandle<SE3>(1).setIsometry(
    Eigen::Isometry3d::Identity());
  
  Eigen::VectorXd value;
  adaptor.getValue(state, value);
  EXPECT_TRUE(value.isApprox(Eigen::VectorXd::Zero(6)));

  Eigen::MatrixXd jacobian;
  adaptor.getJacobian(state, jacobian);
  EXPECT_TRUE(jacobian.isApproxToConstant(0, 1e-3));


  // state outside tsr
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(0, 0, 2);
  state.getSubStateHandle<SE3>(0).setIsometry(isometry);

  adaptor.getJacobian(state, jacobian);

  Eigen::MatrixXd expected(Eigen::MatrixXd::Zero(6, 12));
  expected(2, 5) = 1; 
  expected(2, 11) = -1;
  EXPECT_TRUE(jacobian.isApprox(expected, 1e-3));

}
