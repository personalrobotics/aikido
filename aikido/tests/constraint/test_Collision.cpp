#include <aikido/constraint/CollisionConstraint.hpp>
#include <gtest/gtest.h>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <dart/dart.h>
#include <aikido/statespace/RealVectorStateSpace.hpp>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::constraint::CollisionConstraint;
using aikido::statespace::SO2StateSpace;
using aikido::statespace::SE3StateSpace;

using namespace dart::dynamics;
// using namespace dart::simulation;
using namespace dart::collision;

class CollisionTest : public ::testing::Test
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
                      <VisualAddon, CollisionAddon, DynamicsAddon>(boxShape);

    // Set shapes on the bodies.
    Eigen::Vector3d shape(0.2, 0.2, 0.7);
    std::shared_ptr<BoxShape> box(new BoxShape(shape));

    // Create a shpae node for visualization and collision checking
    auto shapeNode1 = bn1->createShapeNodeWith
                      <VisualAddon, CollisionAddon, DynamicsAddon>(box);

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
  dart::collision::Option mCollisionOption;
  dart::collision::Result mCollisionResult;

  // statespace setup
  MetaSkeletonStateSpacePtr mStateSpace;
};


TEST_F(CollisionTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(CollisionConstraint(nullptr, mCollisionDetector), 
              std::invalid_argument);
}

TEST_F(CollisionTest, ConstructorThrowsOnNullCollisionDetector)
{
  EXPECT_THROW(CollisionConstraint(mStateSpace, nullptr), 
              std::invalid_argument);
}

TEST_F(CollisionTest, GetStateSpaceMatchStateSpace)
{
  CollisionConstraint constraint(mStateSpace, mCollisionDetector);
  EXPECT_EQ(mStateSpace, constraint.getStateSpace());
}

TEST_F(CollisionTest, EmptyCollisionGroup_IsSatisfiedReturnsTrue)
{
  CollisionConstraint constraint(mStateSpace, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();
  mStateSpace->setState(state);

  EXPECT_TRUE(constraint.isSatisfied(state));
}

TEST_F(CollisionTest, AddPairwiseCheckPasses_IsSatisfied)
{
  CollisionConstraint constraint(mStateSpace, mCollisionDetector);
  constraint.addPairwiseCheck(mCollisionGroup1, mCollisionGroup2);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();


  Eigen::VectorXd position(Eigen::VectorXd::Zero(7));
  position(4) = 5;
  mStateSpace->convertPositionsToState(position, state);

  // Eigen::Isometry3d isometry;
  // isometry.translation() = Eigen::Vector3d(5, 0, 0);
  // state.getSubStateHandle<SE3StateSpace>(0).setIsometry(isometry);
  mStateSpace->setState(state);

  EXPECT_TRUE(constraint.isSatisfied(state));
}

TEST_F(CollisionTest, AddPairwiseCheckFails_IsSatisfied)
{
  CollisionConstraint constraint(mStateSpace, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();

  constraint.addPairwiseCheck(mCollisionGroup1, mCollisionGroup3);
  EXPECT_FALSE(constraint.isSatisfied(state));
}

TEST_F(CollisionTest, AddSelfCheckPasses_IsSatisfied)
{
  CollisionConstraint constraint(mStateSpace, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();
  constraint.addSelfCheck(mCollisionGroup1);
  EXPECT_TRUE(constraint.isSatisfied(state));
}


TEST_F(CollisionTest, AddSelfCheckFails_IsSatisfied)
{
  CollisionConstraint constraint(mStateSpace, mCollisionDetector);

  auto state = mStateSpace->getScopedStateFromMetaSkeleton();

  constraint.addSelfCheck(mCollisionGroup3);
  EXPECT_FALSE(constraint.isSatisfied(state));
}

