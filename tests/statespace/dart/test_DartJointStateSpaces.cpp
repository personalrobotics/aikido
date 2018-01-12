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

// TODO: Add tests for other JointStateSpaces with limits
