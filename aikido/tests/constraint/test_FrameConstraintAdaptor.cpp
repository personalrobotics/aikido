#include <aikido/constraint/FrameConstraintAdaptor.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/statespace/MetaSkeletonStateSpace.hpp>
#include <aikido/util/RNG.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::FrameConstraintAdaptor;
using aikido::constraint::TSR;

using namespace aikido::util;
using namespace dart::dynamics;
using namespace aikido::statespace;

class FrameConstraintAdaptorTest : public ::testing::Test {
  protected:
    virtual void SetUp() {

      tsr = std::make_shared<TSR>(
        std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));

      Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
      Bw(2,0) = 1;
      Bw(2,1) = 1;

      tsr->mBw = Bw;

      skeleton = Skeleton::create("manipulator");

      // Root joint
      FreeJoint::Properties properties1;
      properties1.mName = "root joint";
      bn1 = skeleton->createJointAndBodyNodePair<FreeJoint>(
                        nullptr, properties1, 
                        BodyNode::Properties(std::string("body1"))).second;

      // joint 2, body 2
      RevoluteJoint::Properties properties2;
      properties2.mAxis = Eigen::Vector3d::UnitY();
      properties2.mName = "joint 2";
      properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
      bn2 = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
                        bn1, properties2, 
                        BodyNode::Properties(std::string("body2"))).second;

      MetaSkeletonStateSpace space(skeleton);
      spacePtr = std::make_shared<MetaSkeletonStateSpace>(space);
  }

  BodyNodePtr bn1, bn2;
  SkeletonPtr skeleton;
  MetaSkeletonStateSpacePtr spacePtr;
  std::shared_ptr<TSR> tsr ;

};

TEST_F(FrameConstraintAdaptorTest, Constructor)
{
  FrameConstraintAdaptor adaptor(spacePtr, bn2, tsr);         
  EXPECT_EQ(tsr->getConstraintDimension(),
            adaptor.getConstraintDimension());
}


TEST_F(FrameConstraintAdaptorTest, Value)
{
  FrameConstraintAdaptor adaptor(spacePtr, bn2, tsr);
  auto state = spacePtr->getScopedStateFromMetaSkeleton();
  state.getSubStateHandle<SE3StateSpace>(0).setIsometry(Eigen::Isometry3d::Identity());
  state.getSubStateHandle<SO2StateSpace>(1).setAngle(0);
  
  Eigen::VectorXd value = adaptor.getValue(state);

  EXPECT_TRUE(value.isApprox(Eigen::VectorXd::Zero(6)));

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;
  state.getSubStateHandle<SE3StateSpace>(0).setIsometry(isometry);

  value = adaptor.getValue(state);
  
  Eigen::VectorXd expected(Eigen::VectorXd::Zero(6));
  expected(2) = 2; 
  expected(3) = M_PI;
  expected(5) = M_PI;

  EXPECT_TRUE(value.isApprox(expected));
}


TEST_F(FrameConstraintAdaptorTest, Jacobian)
{

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  for(int i = 0; i < 6; ++i)
  {
    Bw(i, 0) = -0.2;
    Bw(i, 1) = 0.2;
  }
  Bw(2,0) = 0.8;
  Bw(2,1) = 1.2;

  tsr->mBw = Bw;

  FrameConstraintAdaptor adaptor(spacePtr, bn1, tsr);

  // state strictly inside tsr
  auto state = spacePtr->getScopedStateFromMetaSkeleton();
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(0, 0, 1);
  state.getSubStateHandle<SE3StateSpace>(0).setIsometry(isometry);
  state.getSubStateHandle<SO2StateSpace>(1).setAngle(0);
  
  Eigen::Vector6d value = adaptor.getValue(state);
  EXPECT_TRUE(value.isApprox(Eigen::VectorXd::Zero(6)));

  Eigen::MatrixXd jacobian = adaptor.getJacobian(state);
  EXPECT_TRUE(jacobian.isApproxToConstant(0, 1e-3));

  // state outside tsr
  isometry.translation() = Eigen::Vector3d(0, 0, 2);
  state.getSubStateHandle<SE3StateSpace>(0).setIsometry(isometry);

  jacobian = adaptor.getJacobian(state);

  Eigen::MatrixXd expected(Eigen::MatrixXd::Zero(6, 7));
  expected(2, 5) = 1; 
  EXPECT_TRUE(jacobian.isApprox(expected, 1e-3));

}
