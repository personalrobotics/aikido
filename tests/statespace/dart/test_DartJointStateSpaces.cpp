#include <gtest/gtest.h>
#include <aikido/statespace/dart/WeldJoint.hpp>

using namespace aikido;

//==============================================================================
TEST(DartJointStateSpaces, WeldJointStateSpace)
{
  auto skeleton = dart::dynamics::Skeleton::create();
  skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();

  auto weldJoint
      = dynamic_cast<dart::dynamics::WeldJoint*>(skeleton->getJoint(0u));
  EXPECT_TRUE(weldJoint != nullptr);

  statespace::dart::WeldJoint weldJointSs(weldJoint);
  EXPECT_EQ(weldJointSs.getDimension(), 0u);

  auto weldJointState = weldJointSs.createState();
  auto stateValue = weldJointState.getValue();
  EXPECT_EQ(stateValue.size(), 0);
}

//==============================================================================
TEST(DartJointStateSpaces, WeldJointStateSpaceProperties)
{
  auto skeleton = dart::dynamics::Skeleton::create();
  skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();

  auto weldJoint
      = dynamic_cast<dart::dynamics::WeldJoint*>(skeleton->getJoint(0u));
  EXPECT_TRUE(weldJoint != nullptr);

  statespace::dart::WeldJoint statespace(weldJoint);

  EXPECT_EQ(weldJoint->getName(), statespace.getProperties().getName());

  EXPECT_EQ(weldJoint->getType(), statespace.getProperties().getType());

  EXPECT_EQ(weldJoint->getNumDofs(), statespace.getProperties().getNumDofs());
}

//==============================================================================
TEST(DartJointStateSpaces, WeldJointStateSpace_CompatibleJoints)
{
  auto skeleton = dart::dynamics::Skeleton::create();
  skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();

  auto j0 = dynamic_cast<dart::dynamics::WeldJoint*>(skeleton->getJoint(0u));
  statespace::dart::WeldJoint statespace(j0);
  EXPECT_TRUE(statespace.isCompatible(j0));
  EXPECT_NO_THROW(statespace.checkCompatibility(j0));

  auto j1 = dynamic_cast<dart::dynamics::WeldJoint*>(skeleton->getJoint(1u));
  EXPECT_FALSE(statespace.isCompatible(j1));
  EXPECT_THROW(statespace.checkCompatibility(j1), std::invalid_argument);
}

// TODO: Add tests for other JointStateSpaces with limits
