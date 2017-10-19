#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/planner/World.hpp>

using std::shared_ptr;
using std::make_shared;

class WorldTest : public ::testing::Test
{
public:
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  WorldTest()
    : skel1{dart::dynamics::Skeleton::create("skel1")}
    , skel2{dart::dynamics::Skeleton::create("skel2")}
    , skel3{dart::dynamics::Skeleton::create("skel3")}
  {
    mWorld = std::make_shared<aikido::planner::World>("test");
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
  EXPECT_EQ(mWorld->getName(), clone1->getName());

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
