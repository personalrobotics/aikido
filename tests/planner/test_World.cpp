#include <dart/dart.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <gtest/gtest.h>

#include <aikido/planner/World.hpp>

using std::make_shared;
using std::shared_ptr;

class WorldTest : public ::testing::Test
{
public:
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  WorldTest()
    : skel1{dart::dynamics::Skeleton::create("skel1")}
    , skel2{dart::dynamics::Skeleton::create("skel2")}
    , skel3{dart::dynamics::Skeleton::create("skel3")}
    , mWorld{aikido::planner::World::create("test")}
  {
  }

  // DART setup
  SkeletonPtr skel1, skel2, skel3;
  aikido::planner::WorldPtr mWorld;
};

TEST_F(WorldTest, NameTest)
{
  EXPECT_EQ("test", mWorld->getName());
  mWorld->setName("different");
  EXPECT_EQ("different", mWorld->getName());
}

TEST_F(WorldTest, AddGetRemoveSkeletons)
{
  // Add Skeletons
  EXPECT_EQ(0, mWorld->getNumSkeletons());
  mWorld->addSkeleton(skel1);
  EXPECT_EQ(1, mWorld->getNumSkeletons());
  mWorld->addSkeleton(skel1);
  EXPECT_EQ(1, mWorld->getNumSkeletons());
  mWorld->addSkeleton(skel2);
  EXPECT_EQ(2, mWorld->getNumSkeletons());

  // Getting Skeletons by index
  EXPECT_EQ(skel1, mWorld->getSkeleton(0));
  EXPECT_EQ(skel2, mWorld->getSkeleton(1));
  EXPECT_TRUE(mWorld->getSkeleton(2) == nullptr);

  // Getting Skeletons by name
  EXPECT_EQ(skel1, mWorld->getSkeleton("skel1"));
  EXPECT_EQ(skel2, mWorld->getSkeleton("skel2"));
  EXPECT_TRUE(mWorld->getSkeleton("skel3") == nullptr);

  // Remove Skeletons
  mWorld->removeSkeleton(skel3);
  EXPECT_EQ(2, mWorld->getNumSkeletons());
  mWorld->removeSkeleton(skel2);
  EXPECT_EQ(1, mWorld->getNumSkeletons());
  mWorld->removeSkeleton(skel2);
  EXPECT_EQ(1, mWorld->getNumSkeletons());
  mWorld->removeSkeleton(skel1);
  EXPECT_EQ(0, mWorld->getNumSkeletons());
}

TEST_F(WorldTest, CloningPreservesSkeletonNamesAndConfigurations)
{
  mWorld->addSkeleton(skel1);
  mWorld->addSkeleton(skel2);
  mWorld->addSkeleton(skel3);

  auto clone1 = mWorld->clone();
  EXPECT_NE(mWorld->getName(), clone1->getName());

  auto clone2 = clone1->clone("clone2");
  EXPECT_EQ("clone2", clone2->getName());

  EXPECT_EQ(mWorld->getNumSkeletons(), clone1->getNumSkeletons());
  EXPECT_EQ(mWorld->getNumSkeletons(), clone2->getNumSkeletons());
  for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i)
  {
    auto origSkeleton = mWorld->getSkeleton(i);
    auto clonedSkeleton1 = clone1->getSkeleton(i);
    auto clonedSkeleton2 = clone2->getSkeleton(i);

    EXPECT_EQ(origSkeleton->getName(), clonedSkeleton1->getName());
    EXPECT_EQ(origSkeleton->getName(), clonedSkeleton2->getName());

    EXPECT_EQ(
        origSkeleton->getConfiguration(), clonedSkeleton1->getConfiguration());
    EXPECT_EQ(
        origSkeleton->getConfiguration(), clonedSkeleton2->getConfiguration());
  }
}

TEST_F(WorldTest, EqualStatesReturnsTrueForEmptyWorlds)
{
  auto otherWorld = aikido::planner::World::create("other");
  EXPECT_TRUE(mWorld->getState() == otherWorld->getState());
  EXPECT_TRUE(otherWorld->getState() == mWorld->getState());
}

TEST_F(WorldTest, EqualStatesReturnsTrueForClonedWorlds)
{
  mWorld->addSkeleton(skel1);
  mWorld->addSkeleton(skel2);
  mWorld->addSkeleton(skel3);

  auto otherWorld = mWorld->clone();
  EXPECT_TRUE(mWorld->getState() == otherWorld->getState());
  EXPECT_TRUE(otherWorld->getState() == mWorld->getState());
}

TEST_F(
    WorldTest, EqualStatesReturnsFalseForWorldsWithDifferentNumberOfSkeletons)
{
  auto otherWorld = aikido::planner::World::create("other");
  mWorld->addSkeleton(skel1);

  EXPECT_FALSE(mWorld->getState() == otherWorld->getState());
  EXPECT_FALSE(otherWorld->getState() == mWorld->getState());
}

TEST_F(
    WorldTest,
    EqualStatesReturnsFalseForWorldsWithSkeletonsWithDifferentJointValues)
{
  using dart::dynamics::RevoluteJoint;

  mWorld->addSkeleton(skel1);
  skel1->createJointAndBodyNodePair<RevoluteJoint>();
  skel1->setPosition(0, 0.5);

  auto otherWorld = mWorld->clone();
  otherWorld->getSkeleton(0)->setPosition(0, 1.0);

  EXPECT_FALSE(mWorld->getState() == otherWorld->getState());
  EXPECT_FALSE(otherWorld->getState() == mWorld->getState());
}

TEST_F(WorldTest, SetStateThrowsErrorsOnWorldsWithDifferentNumberOfSkeletons)
{
  mWorld->addSkeleton(skel1);
  mWorld->addSkeleton(skel2);

  auto state = mWorld->getState();
  mWorld->addSkeleton(skel3);

  EXPECT_THROW(mWorld->setState(state), std::invalid_argument);
}

TEST_F(WorldTest, SetStateThrowsErrorsOnWorldsWithSkeletonsWithDifferentNames)
{
  mWorld->addSkeleton(skel1);

  auto clonedWorld = mWorld->clone();
  auto state = clonedWorld->getState();
  EXPECT_NO_THROW(mWorld->setState(state));

  auto skel = clonedWorld->getSkeleton(0);
  clonedWorld->removeSkeleton(skel);
  skel->setName("testSkel");
  clonedWorld->addSkeleton(skel);
  state = clonedWorld->getState();
  EXPECT_THROW(mWorld->setState(state), std::invalid_argument);
}
