#include <aikido/constraint/NonColliding.hpp>
#include <gtest/gtest.h>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <dart/dart.hpp>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::constraint::NonColliding;
using aikido::statespace::SO2;
using aikido::statespace::SE3;

using namespace dart::dynamics;
using namespace dart::collision;

class NonCollidingTest : public ::testing::Test
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
    auto bn1 = mManipulator->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties1).second;

    // Box
    mBox = Skeleton::create("Box");
    auto boxNode = mBox->createJointAndBodyNodePair<FreeJoint>().second;
    Eigen::Vector3d boxSize(0.5, 0.5, 0.5);
    std::shared_ptr<BoxShape> boxShape(new BoxShape(boxSize));
    auto shapeNode = boxNode->createShapeNodeWith
                      <VisualAspect, CollisionAspect, DynamicsAspect>(boxShape);

    // Set shapes on the bodies.
    Eigen::Vector3d shape(0.2, 0.2, 0.7);
    std::shared_ptr<BoxShape> box(new BoxShape(shape));

    // Create a shpae node for visualization and collision checking
    auto shapeNode1 = bn1->createShapeNodeWith
                      <VisualAspect, CollisionAspect, DynamicsAspect>(box);

    // Add skeleton to world
    mCollisionDetector = FCLCollisionDetector::create();

    // Create 3 collision groups
    mCollisionGroup1 = mCollisionDetector->createCollisionGroup(bn1);
    mCollisionGroup2 = mCollisionDetector->createCollisionGroup(boxNode);
    mCollisionGroup3 = mCollisionDetector->createCollisionGroup(bn1, boxNode);

    // StateSpace for controlling skeleton
    GroupPtr group = Group::create();
    group->addBodyNode(bn1);
    group->addBodyNode(boxNode);
    group->addDofs(mManipulator->getDofs());
    group->addDofs(mBox->getDofs());

    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(group);
    mManipulator->enableSelfCollision(true);

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
};


TEST_F(NonCollidingTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(NonColliding(nullptr, mCollisionDetector), 
              std::invalid_argument);
}

TEST_F(NonCollidingTest, ConstructorThrowsOnNullCollisionDetector)
{
  EXPECT_THROW(NonColliding(mStateSpace, nullptr), 
              std::invalid_argument);
}

TEST_F(NonCollidingTest, GetStateSpaceMatchStateSpace)
{
  NonColliding constraint(mStateSpace, mCollisionDetector);
  EXPECT_EQ(mStateSpace, constraint.getStateSpace());
}

TEST_F(NonCollidingTest, EmptyCollisionGroup_IsSatisfiedReturnsTrue)
{
  NonColliding constraint(mStateSpace, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();
  mStateSpace->setState(state);

  EXPECT_TRUE(constraint.isSatisfied(state));
}

TEST_F(NonCollidingTest, AddPairwiseCheckPasses_IsSatisfied)
{
  NonColliding constraint(mStateSpace, mCollisionDetector);
  constraint.addPairwiseCheck(mCollisionGroup1, mCollisionGroup2);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();
  
  Eigen::VectorXd position(Eigen::VectorXd::Zero(7));
  position(4) = 5;
  mStateSpace->convertPositionsToState(position, state);
  mStateSpace->setState(state);

  EXPECT_TRUE(constraint.isSatisfied(state));
}

TEST_F(NonCollidingTest, AddPairwiseCheckFails_IsSatisfied)
{
  NonColliding constraint(mStateSpace, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();

  constraint.addPairwiseCheck(mCollisionGroup1, mCollisionGroup3);
  EXPECT_FALSE(constraint.isSatisfied(state));
}

TEST_F(NonCollidingTest, AddSelfCheckPasses_IsSatisfied)
{
  NonColliding constraint(mStateSpace, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();
  constraint.addSelfCheck(mCollisionGroup1);
  EXPECT_TRUE(constraint.isSatisfied(state));
}


TEST_F(NonCollidingTest, AddSelfCheckFails_IsSatisfied)
{
  NonColliding constraint(mStateSpace, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();

  constraint.addSelfCheck(mCollisionGroup3);
  EXPECT_FALSE(constraint.isSatisfied(state));
}

