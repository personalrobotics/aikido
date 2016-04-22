#include <random>
#include <aikido/constraint/FkTestable.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::FkTestable;
using aikido::constraint::SampleGenerator;
using aikido::constraint::TSR;
using aikido::statespace::SO2;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::util::RNG;
using aikido::util::RNGWrapper;
using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;

class FkTestableTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto robot = Skeleton::create("robot");

    // Root joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitZ();
    properties1.mName = "j1";
    auto bn1 = robot->createJointAndBodyNodePair<RevoluteJoint>(
                          nullptr, properties1,
                          BodyNode::Properties(std::string("b1"))).second;

    // Joint 2
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitZ();
    properties2.mName = "j2";
    properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 1, 0);
    auto bn2 = robot->createJointAndBodyNodePair<RevoluteJoint>(
                          bn1, properties2,
                          BodyNode::Properties(std::string("b2"))).second;

    // End effector
    RevoluteJoint::Properties propertiesEE;
    propertiesEE.mAxis = Eigen::Vector3d::UnitZ();
    propertiesEE.mName = "ee";
    propertiesEE.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 1, 0);
    endEffector = robot->createJointAndBodyNodePair<RevoluteJoint>(
                             bn2, propertiesEE,
                             BodyNode::Properties(std::string("b3"))).second;

    // Statespace
    stateSpace = std::make_shared<MetaSkeletonStateSpace>(robot);
  }

  BodyNodePtr endEffector;
  MetaSkeletonStateSpacePtr stateSpace;
};

TEST_F(FkTestableTest, StateSpaceEquality)
{
  auto tsr = std::make_shared<TSR>();
  FkTestable testable(stateSpace, endEffector.get(), tsr);

  EXPECT_EQ(stateSpace.get(), testable.getStateSpace().get());
}

TEST_F(FkTestableTest, PoseTests)
{
  auto T0_w = Eigen::Isometry3d::Identity();
  auto Tw_e = Eigen::Isometry3d::Identity();
  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(0, 0) = 1;
  Bw(0, 1) = 2;
  Bw(1, 0) = -0.5;
  Bw(1, 1) = 0.5;
  Bw(3, 0) = -M_PI;
  Bw(3, 1) = M_PI;
  Bw(4, 0) = -M_PI;
  Bw(4, 1) = M_PI;
  Bw(5, 0) = -M_PI;
  Bw(5, 1) = M_PI;

  auto tsr = std::make_shared<TSR>(T0_w, Bw, Tw_e);

  FkTestable testable(stateSpace, endEffector.get(), tsr);

  auto state = stateSpace->createState();
  auto j1Joint = stateSpace->getSubState<SO2::State>(state, 0);
  auto j2Joint = stateSpace->getSubState<SO2::State>(state, 1);

  j1Joint->setAngle(-M_PI / 4);
  j2Joint->setAngle(-M_PI / 2);
  EXPECT_TRUE(testable.isSatisfied(state));

  j2Joint->setAngle(0);
  EXPECT_FALSE(testable.isSatisfied(state));

  j1Joint->setAngle(-0.75 * M_PI);
  j2Joint->setAngle(M_PI/2);
  EXPECT_TRUE(testable.isSatisfied(state));
}
