#include <dart/dynamics/dynamics.hpp>
#include <gtest/gtest.h>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>

using dart::dynamics::Skeleton;
using dart::dynamics::RevoluteJoint;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpaceSaver;
using aikido::statespace::SO2;

TEST(MetaSkeletonStateSpaceSaver, MetaSkeletonStateSpaceReturnsToOriginal)
{

  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<RevoluteJoint>();

  auto space = std::make_shared<MetaSkeletonStateSpace>(skeleton);
  ASSERT_EQ(1, space->getNumSubspaces());

  auto state = space->createState();
  auto substate = state.getSubStateHandle<SO2>(0);

  substate.setAngle(1.);
  space->setState(state);
  EXPECT_DOUBLE_EQ(1., skeleton->getPosition(0));

  {
    auto saver = MetaSkeletonStateSpaceSaver(space);
    DART_UNUSED(saver);

    substate.setAngle(6.);
    space->setState(state);
    EXPECT_DOUBLE_EQ(6., skeleton->getPosition(0));
  }
  EXPECT_DOUBLE_EQ(1., skeleton->getPosition(0));
}
