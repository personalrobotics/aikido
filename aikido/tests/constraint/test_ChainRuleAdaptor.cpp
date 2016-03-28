#include <aikido/constraint/ChainRuleAdaptor.hpp>
#include <aikido/constraint/TSR.hpp>

#include <aikido/state/State.hpp>
#include <aikido/util/RNG.hpp>

#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::constraint::ChainRuleAdaptor;
using aikido::constraint::TSRConstraint;
using aikido::constraint::TSR;

using namespace aikido::util;
using namespace aikido::state;
using namespace dart::dynamics;



class ChainRuleAdaptorTest : public ::testing::Test {
  protected:
    virtual void SetUp() {

      std::shared_ptr<TSR> tsr = std::make_shared<TSR>(
        std::unique_ptr<RNG>(new RNGWrapper<std::default_random_engine>(0)));

      Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
      Bw(2,0) = 1;
      Bw(2,1) = 1;

      tsr->mBw = Bw;

      tsrConstraint = std::make_shared<TSRConstraint>(tsr);

      skeleton = Skeleton::create("manipulator");

      // Root joint
      RevoluteJoint::Properties properties1;
      properties1.mAxis = Eigen::Vector3d::UnitY();
      properties1.mName = "root joint";
      bn1 = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
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

      ChainRuleAdaptor::BodySkeletonPair p(bn2, skeleton);

      pairs.push_back(p);

         
  }

  std::shared_ptr<TSRConstraint> tsrConstraint;
  std::vector<ChainRuleAdaptor::BodySkeletonPair> pairs;
  BodyNodePtr bn1, bn2;
  SkeletonPtr skeleton;

};

TEST_F(ChainRuleAdaptorTest, Constructor)
{
  ChainRuleAdaptor adaptor(tsrConstraint, pairs);
  EXPECT_EQ(tsrConstraint->getConstraintDimension(),
            adaptor.getConstraintDimension());
}



TEST_F(ChainRuleAdaptorTest, Value)
{
  ChainRuleAdaptor adaptor(tsrConstraint, pairs);
  RealVectorStatePtr rvState = std::make_shared<RealVectorState>(
                                Eigen::Vector2d(0, 0));
  std::vector<StatePtr> comps; 
  comps.push_back(rvState);
  CompoundStatePtr state = std::make_shared<CompoundState>(comps);
  Eigen::VectorXd val = adaptor.getValue(state);

  EXPECT_TRUE(val.isApprox(Eigen::VectorXd::Zero(6)));

  rvState->mQ(0) = M_PI;
  val = adaptor.getValue(state);
  
  Eigen::VectorXd expected(Eigen::VectorXd::Zero(6));
  expected(2) = 2; 
  expected(3) = M_PI;
  expected(5) = M_PI;

  EXPECT_TRUE(val.isApprox(expected));

}


TEST_F(ChainRuleAdaptorTest, Jacobian)
{
  ChainRuleAdaptor adaptor(tsrConstraint, pairs);
  RealVectorStatePtr rvState = std::make_shared<RealVectorState>(
                                Eigen::Vector2d(0, 0));
  std::vector<StatePtr> comps; 
  comps.push_back(rvState);
  CompoundStatePtr state = std::make_shared<CompoundState>(comps);
  JacobianPtr _jac = adaptor.getJacobian(state);
  CompoundJacobianPtr jac = std::dynamic_pointer_cast<CompoundJacobian>(_jac);

  RealVectorJacobianPtr rvJac = std::dynamic_pointer_cast
                                  <RealVectorJacobian>(jac->mJacobian.at(0));
  Eigen::MatrixXd expected = bn2->getJacobian();

  EXPECT_TRUE(rvJac->mJacobian.isApprox(bn2->getJacobian()));
}
