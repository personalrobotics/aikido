#include <dart/dart.hpp>
#include <gtest/gtest.h>

#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

using aikido::constraint::TestableOutcome;
using aikido::constraint::dart::CollisionFree;
using aikido::constraint::dart::CollisionFreeOutcome;
using aikido::statespace::SE3;
using aikido::statespace::SO2;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;

using namespace dart::dynamics;
using namespace dart::collision;

class CollisionFreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Manipulator with 1 joint
    mManipulator = Skeleton::create("Manipulator");

    // Root free joint, body 1
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitY();
    properties1.mName = "Joint1";
    auto bn1
        = mManipulator
              ->createJointAndBodyNodePair<RevoluteJoint>(nullptr, properties1)
              .second;

    // Box
    mBox = Skeleton::create("Box");
    auto boxNode = mBox->createJointAndBodyNodePair<FreeJoint>().second;
    Eigen::Vector3d boxSize(0.5, 0.5, 0.5);
    std::shared_ptr<BoxShape> boxShape(new BoxShape(boxSize));
    boxNode->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
        boxShape);

    // Set shapes on the bodies.
    Eigen::Vector3d shape(0.2, 0.2, 0.7);
    std::shared_ptr<BoxShape> box(new BoxShape(shape));
    bn1->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
        box);

    // Add skeleton to world
    mCollisionDetector = FCLCollisionDetector::create();

    // Create 3 collision groups
    mCollisionGroup1 = mCollisionDetector->createCollisionGroup(bn1);
    mCollisionGroup2 = mCollisionDetector->createCollisionGroup(boxNode);
    mCollisionGroup3 = mCollisionDetector->createCollisionGroup(bn1, boxNode);

    // StateSpace for controlling skeleton
    mSkeleton = Group::create();
    mSkeleton->addBodyNode(bn1);
    mSkeleton->addBodyNode(boxNode);
    mSkeleton->addDofs(mManipulator->getDofs());
    mSkeleton->addDofs(mBox->getDofs());

    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(mSkeleton.get());
    mManipulator->enableSelfCollisionCheck();
    mManipulator->enableAdjacentBodyCheck();
  }

public:
  // dart setup
  SkeletonPtr mManipulator, mBox;
  CollisionDetectorPtr mCollisionDetector;
  std::shared_ptr<CollisionGroup> mCollisionGroup1;
  std::shared_ptr<CollisionGroup> mCollisionGroup2;
  std::shared_ptr<CollisionGroup> mCollisionGroup3;
  dart::collision::CollisionOption mCollisionOption;
  dart::collision::CollisionResult mCollisionResult;

  // statespace setup
  MetaSkeletonStateSpacePtr mStateSpace;
  dart::dynamics::GroupPtr mSkeleton;
};

TEST_F(CollisionFreeTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(
      CollisionFree(nullptr, nullptr, mCollisionDetector),
      std::invalid_argument);
}

TEST_F(CollisionFreeTest, ConstructorThrowsOnNullCollisionDetector)
{
  EXPECT_THROW(
      CollisionFree(mStateSpace, mSkeleton, nullptr), std::invalid_argument);
}

TEST_F(CollisionFreeTest, GetStateSpaceMatchStateSpace)
{
  CollisionFree constraint(mStateSpace, mSkeleton, mCollisionDetector);
  EXPECT_EQ(mStateSpace, constraint.getStateSpace());
}

TEST_F(CollisionFreeTest, EmptyCollisionGroup_IsSatisfiedReturnsTrue)
{
  CollisionFree constraint(mStateSpace, mSkeleton, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());
  mStateSpace->setState(mSkeleton.get(), state);

  std::unique_ptr<TestableOutcome> outcome = constraint.createOutcome();
  EXPECT_TRUE(constraint.isSatisfied(state, outcome.get()));
  EXPECT_TRUE(outcome->isSatisfied());
}

TEST_F(CollisionFreeTest, AddPairwiseCheckPasses_IsSatisfied)
{
  CollisionFree constraint(mStateSpace, mSkeleton, mCollisionDetector);
  constraint.addPairwiseCheck(mCollisionGroup1, mCollisionGroup2);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());

  Eigen::VectorXd position(Eigen::VectorXd::Zero(7));
  position(4) = 5;
  mStateSpace->convertPositionsToState(position, state);
  mStateSpace->setState(mSkeleton.get(), state);

  std::unique_ptr<TestableOutcome> outcome = constraint.createOutcome();
  EXPECT_TRUE(constraint.isSatisfied(state, outcome.get()));
  EXPECT_TRUE(outcome->isSatisfied());
}

TEST_F(CollisionFreeTest, AddPairwiseCheckFails_IsSatisfied)
{
  CollisionFree constraint(mStateSpace, mSkeleton, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());

  constraint.addPairwiseCheck(mCollisionGroup1, mCollisionGroup3);
  CollisionFreeOutcome outcome;
  EXPECT_FALSE(constraint.isSatisfied(state, &outcome));
  EXPECT_FALSE(outcome.isSatisfied());
  EXPECT_EQ(0, outcome.getSelfContacts().size());

  auto pairwiseContacts = outcome.getPairwiseContacts();
  EXPECT_EQ(1, pairwiseContacts.size());
  EXPECT_EQ(
      "BodyNode",
      outcome.getCollisionObjectName(
          pairwiseContacts.front().collisionObject1));
  EXPECT_EQ(
      "BodyNode",
      outcome.getCollisionObjectName(
          pairwiseContacts.front().collisionObject2));
}

TEST_F(CollisionFreeTest, AddAndRemovePairwiseCheckFails_IsSatisfied)
{
  CollisionFree constraint(mStateSpace, mSkeleton, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());
  // Also reuse this outcome object and ensure the correct result
  // is still returned,
  CollisionFreeOutcome outcome;

  constraint.addPairwiseCheck(mCollisionGroup1, mCollisionGroup3);
  EXPECT_FALSE(constraint.isSatisfied(state, &outcome));
  EXPECT_FALSE(outcome.isSatisfied());

  constraint.removePairwiseCheck(mCollisionGroup1, mCollisionGroup3);
  EXPECT_TRUE(constraint.isSatisfied(state, &outcome));
  EXPECT_TRUE(outcome.isSatisfied());

  constraint.addPairwiseCheck(mCollisionGroup1, mCollisionGroup3);
  EXPECT_FALSE(constraint.isSatisfied(state, &outcome));
  EXPECT_FALSE(outcome.isSatisfied());

  constraint.removePairwiseCheck(mCollisionGroup3, mCollisionGroup1);
  EXPECT_TRUE(constraint.isSatisfied(state, &outcome));
  EXPECT_TRUE(outcome.isSatisfied());
}

TEST_F(CollisionFreeTest, AddSelfCheckPasses_IsSatisfied)
{
  CollisionFree constraint(mStateSpace, mSkeleton, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());
  constraint.addSelfCheck(mCollisionGroup1);

  std::unique_ptr<TestableOutcome> outcome = constraint.createOutcome();
  EXPECT_TRUE(constraint.isSatisfied(state, outcome.get()));
  EXPECT_TRUE(outcome->isSatisfied());
}

TEST_F(CollisionFreeTest, AddSelfCheckFails_IsSatisfied)
{
  CollisionFree constraint(mStateSpace, mSkeleton, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());

  constraint.addSelfCheck(mCollisionGroup3);
  CollisionFreeOutcome outcome;
  EXPECT_FALSE(constraint.isSatisfied(state, &outcome));
  EXPECT_FALSE(outcome.isSatisfied());
  EXPECT_EQ(0, outcome.getPairwiseContacts().size());

  auto selfContacts = outcome.getSelfContacts();
  EXPECT_EQ(1, selfContacts.size());
  EXPECT_EQ(
      "BodyNode",
      outcome.getCollisionObjectName(selfContacts.front().collisionObject1));
  EXPECT_EQ(
      "BodyNode",
      outcome.getCollisionObjectName(selfContacts.front().collisionObject2));
}

TEST_F(CollisionFreeTest, AddAndRemoveSelfCheckFails_IsSatisfied)
{
  CollisionFree constraint(mStateSpace, mSkeleton, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());

  constraint.addSelfCheck(mCollisionGroup3);
  EXPECT_FALSE(constraint.isSatisfied(state));

  constraint.removeSelfCheck(mCollisionGroup3);
  EXPECT_TRUE(constraint.isSatisfied(state));
}
