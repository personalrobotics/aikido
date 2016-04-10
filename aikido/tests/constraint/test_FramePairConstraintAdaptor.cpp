#include <aikido/constraint/FramePairConstraintAdaptor.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/statespace/MetaSkeletonStateSpace.hpp>
#include <aikido/util/RNG.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::FramePairConstraintAdaptor;
using aikido::constraint::TSR;

using namespace aikido::util;
using namespace dart::dynamics;
using namespace aikido::statespace;

class FramePairConstraintAdaptorTest : public ::testing::Test {
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

TEST_F(FramePairConstraintAdaptorTest, Constructor)
{
  FramePairConstraintAdaptor adaptor(spacePtr, bn1, bn2, tsr);         
  EXPECT_EQ(tsr->getConstraintDimension(),
            adaptor.getConstraintDimension());
}


TEST_F(FramePairConstraintAdaptorTest, Value)
{
  FramePairConstraintAdaptor adaptor(spacePtr, bn1, bn2, tsr);
  auto state = spacePtr->getScopedStateFromMetaSkeleton();
  state.getSubStateHandle<SE3StateSpace>(0).setIsometry(
    Eigen::Isometry3d::Identity());
  state.getSubStateHandle<SE3StateSpace>(1).setIsometry(
    Eigen::Isometry3d::Identity());
  
  Eigen::VectorXd value = adaptor.getValue(state);
  EXPECT_TRUE(value.isApprox(Eigen::VectorXd::Zero(6)));

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(2,0) = 1;
  Bw(2,1) = 1;

  tsr->mBw = Bw;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  value = adaptor.getValue(state);
  
  Eigen::VectorXd expected(Eigen::VectorXd::Zero(6));
  expected(2) = 1;
  EXPECT_TRUE(value.isApprox(expected));
}

TEST_F(FramePairConstraintAdaptorTest, Jacobian)
{

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  for(int i = 0; i < 6; ++i)
  {
    Bw(i, 0) = -0.2;
    Bw(i, 1) = 0.2;
  }

  tsr->mBw = Bw;

  FramePairConstraintAdaptor adaptor(spacePtr, bn1, bn2, tsr);

  // state strictly inside tsr
  auto state = spacePtr->getScopedStateFromMetaSkeleton();
  state.getSubStateHandle<SE3StateSpace>(0).setIsometry(
    Eigen::Isometry3d::Identity());
  state.getSubStateHandle<SE3StateSpace>(1).setIsometry(
    Eigen::Isometry3d::Identity());
  
  Eigen::Vector6d value = adaptor.getValue(state);
  EXPECT_TRUE(value.isApprox(Eigen::VectorXd::Zero(6)));

  Eigen::MatrixXd jacobian = adaptor.getJacobian(state);
  EXPECT_TRUE(jacobian.isApproxToConstant(0, 1e-3));


  // state outside tsr
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(0, 0, 2);
  state.getSubStateHandle<SE3StateSpace>(0).setIsometry(isometry);

  jacobian = adaptor.getJacobian(state);

  Eigen::MatrixXd expected(Eigen::MatrixXd::Zero(6, 12));
  expected(2, 5) = 1; 
  expected(2, 11) = -1;
  EXPECT_TRUE(jacobian.isApprox(expected, 1e-3));

}
