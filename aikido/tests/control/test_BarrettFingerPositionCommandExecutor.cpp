#include <gtest/gtest.h>
#include <aikido/control/BarrettFingerPositionCommandExecutor.hpp>
#include <dart/dart.h>
#include <chrono>

using aikido::control::BarrettFingerPositionCommandExecutor;
using ::dart::dynamics::Chain;
using ::dart::dynamics::ChainPtr;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;

using namespace dart::dynamics;
using namespace dart::collision;

class BarrettFingerPositionCommandExecutorTest : public testing::Test
{
public:

  void setGeometry(const BodyNodePtr& bn)
  {
    // Create a BoxShape to be used for both visualization and collision checking
    Eigen::Vector3d fingerSize(0.1, 0.1, 0.9);
    std::shared_ptr<BoxShape> fingerShape(new BoxShape(fingerSize));

    // Create a shpae node for visualization and collision checking
    auto shapeNode
        = bn->createShapeNodeWith<VisualAddon, CollisionAddon, DynamicsAddon>(fingerShape);

    // Set the location of the shape node
    Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
    Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0.9 / 2.0);
    box_tf.translation() = center;
    shapeNode->setRelativeTransform(box_tf);

    // Move the center of mass to the center of the object
    bn->setLocalCOM(center);
  }

  ChainPtr create3DoFFinger(Eigen::Isometry3d transform
    = Eigen::Isometry3d::Identity())
  {
    mFinger = Skeleton::create("Finger");

    Eigen::Vector3d fingerSize(0.1, 0.1, 0.9);
    std::shared_ptr<BoxShape> fingerShape(new BoxShape(fingerSize));

    // spread joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitX();
    properties1.mName = "Joint1";
    mBn1 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties1, 
      BodyNode::Properties(std::string("spread"))).second;
    mBn1->getParentJoint()->setTransformFromParentBodyNode(transform);

    // primal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn1, properties2, 
      BodyNode::Properties(std::string("primal"))).second;
    setGeometry(mBn2);

    // distal joint
    RevoluteJoint::Properties properties3;
    properties3.mAxis = Eigen::Vector3d::UnitY();
    properties3.mName = "Joint3";
    properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    mBn3 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn2, properties3, 
      BodyNode::Properties(std::string("distal"))).second;
    setGeometry(mBn3);

    Chain::IncludeBoth_t t;
    return Chain::create(mBn1, mBn3, t);
  }

  ChainPtr create2DoFFinger(Eigen::Isometry3d transform
    = Eigen::Isometry3d::Identity())
  {
    mFinger = Skeleton::create("Finger");

    Eigen::Vector3d fingerSize(0.1, 0.1, 0.7);
    std::shared_ptr<BoxShape> fingerShape(new BoxShape(fingerSize));

    // primal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    mBn1 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties2, 
      BodyNode::Properties(std::string("primal"))).second;
    mBn1->createShapeNodeWith<VisualAddon, CollisionAddon, DynamicsAddon>(
      fingerShape);
    mBn1->getParentJoint()->setTransformFromParentBodyNode(transform);
    setGeometry(mBn1);

    // distal joint
    RevoluteJoint::Properties properties3;
    properties3.mAxis = Eigen::Vector3d::UnitY();
    properties3.mName = "Joint3";
    properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn1, properties3, 
      BodyNode::Properties(std::string("distal"))).second;
    setGeometry(mBn2);

    Chain::IncludeBoth_t t;
    return Chain::create(mBn1, mBn2, t);
  }

  /// \param transform pose of ball
  /// \param collisionDetector CollisionDetector to create CollisionGroup with
  BodyNodePtr createBall(Eigen::Isometry3d transform,
    ::dart::collision::CollisionDetectorPtr collisionDetector)
  {
    std::shared_ptr<EllipsoidShape> ballShape(
      new EllipsoidShape(Eigen::Vector3d(0.1, 0.1, 0.1)));
    
    auto skeleton = Skeleton::create("Ball");
    auto ballBody = skeleton->createJointAndBodyNodePair<FreeJoint>(
      nullptr).second;
      ballBody->createShapeNodeWith<VisualAddon, CollisionAddon, DynamicsAddon>(
        ballShape);
    ballBody->getParentJoint()->setTransformFromParentBodyNode(
      transform);

    return ballBody;
  }

  virtual void SetUp()
  {
    // mFinger = Skeleton::create("Finger");

    // // spread joint
    // RevoluteJoint::Properties properties1;
    // properties1.mAxis = Eigen::Vector3d::UnitZ();
    // properties1.mName = "Joint1";
    // mBn1 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
    //   nullptr, properties1, 
    //   BodyNode::Properties(std::string("spread"))).second;

    // // primal joint
    // RevoluteJoint::Properties properties2;
    // properties2.mAxis = Eigen::Vector3d::UnitY();
    // properties2.mName = "Joint2";
    // mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
    //   mBn1, properties2, 
    //   BodyNode::Properties(std::string("primal"))).second;

    // // distal joint
    // RevoluteJoint::Properties properties3;
    // properties3.mAxis = Eigen::Vector3d::UnitY();
    // properties3.mName = "Joint3";
    // properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    // mBn3 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
    //   mBn2, properties3, 
    //   BodyNode::Properties(std::string("distal"))).second;

    mFingerChain = create3DoFFinger(Eigen::Isometry3d::Identity());

    // CollisionDetector
    mCollisionDetector = FCLCollisionDetector::create();
    mCollideWith = mCollisionDetector->createCollisionGroupAsSharedPtr();
 
    mPosition = 1;
    mPrimalDof = 1;
    mDistalDof = 2;
  }

protected:
  SkeletonPtr mFinger;
  ChainPtr mFingerChain;
  BodyNodePtr mBn1, mBn2, mBn3;

  int mPrimalDof, mDistalDof;

  std::chrono::milliseconds mDuration;
  CollisionDetectorPtr mCollisionDetector;
  CollisionGroupPtr mCollideWith;

  double mPosition;
  static constexpr double eps = 1e-2;
};


TEST_F(BarrettFingerPositionCommandExecutorTest, constructor_NullChain_Throws)
{
  EXPECT_THROW(BarrettFingerPositionCommandExecutor(
    nullptr, mPrimalDof, mDistalDof, mCollisionDetector),
    std::invalid_argument);
}

TEST_F(BarrettFingerPositionCommandExecutorTest, constructor)
{
  EXPECT_NO_THROW(BarrettFingerPositionCommandExecutor(
    mFingerChain, mPrimalDof, mDistalDof, mCollisionDetector));
}

TEST_F(BarrettFingerPositionCommandExecutorTest, constructor_NonexistingPrimal_throws)
{
  int primalDof = 3;
  EXPECT_THROW(BarrettFingerPositionCommandExecutor(
    mFingerChain, primalDof, mDistalDof, mCollisionDetector),
    std::invalid_argument);
}

TEST_F(BarrettFingerPositionCommandExecutorTest, constructor_NonexistingDistal_throws)
{
  int distalDof = 3;
  EXPECT_THROW(BarrettFingerPositionCommandExecutor(
    mFingerChain, mPrimalDof, distalDof, mCollisionDetector),
    std::invalid_argument);
}

TEST_F(BarrettFingerPositionCommandExecutorTest, execute_WaitOnFuture_CommandExecuted)
{
  BarrettFingerPositionCommandExecutor executor(
    mFingerChain, mPrimalDof, mDistalDof, mCollisionDetector);

  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalPrimal = mPosition;
  double goalDistal = mPosition*mimicRatio;

  auto future = executor.execute(mPosition, mCollideWith);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.wait();

  EXPECT_NEAR(goalPrimal, mBn2->getParentJoint()->getDof(0)->getPosition(), eps);
  EXPECT_NEAR(goalDistal, mBn3->getParentJoint()->getDof(0)->getPosition(), eps);
}

TEST_F(BarrettFingerPositionCommandExecutorTest, execute_PrimalStopsAtCollsionDistalContinues)
{
  // Collision obstacle
  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
  transform.translation() = Eigen::Vector3d(0.3, 0, 0.3);
  auto ball = createBall(transform, mCollisionDetector);
  auto collideWith = mCollisionDetector->createCollisionGroup(ball);

  double goal = M_PI;

  BarrettFingerPositionCommandExecutor executor(
    mFingerChain, mPrimalDof, mDistalDof, mCollisionDetector);

  auto future = executor.execute(goal, std::move(collideWith));

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.wait();

  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  double primalExpected = 0.56548; // Value made by visual inspection
  double primalActual = mBn2->getParentJoint()->getDof(0)->getPosition();
  double distalExpected = goal*mimicRatio;
  double distalActual = mBn3->getParentJoint()->getDof(0)->getPosition();

  EXPECT_NEAR(primalExpected, primalActual, eps);
  EXPECT_NEAR(distalExpected, distalActual, eps);
}


TEST_F(BarrettFingerPositionCommandExecutorTest, execute_DistalStopsAtCollsionPromalAlsoStops)
{
  // Collision object
  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
  transform.translation() = Eigen::Vector3d(1.3, 0, 1.3);
  auto ball = createBall(transform, mCollisionDetector);
  auto collideWith = mCollisionDetector->createCollisionGroup(ball);

  // Executor
  BarrettFingerPositionCommandExecutor executor(
    mFingerChain, mPrimalDof, mDistalDof, mCollisionDetector);
  double goal = M_PI/4;

  // Execute
  auto future = executor.execute(goal, std::move(collideWith));
  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.wait();

  // Values made by visual inspection
  double primalExpected = 0.636173;
  double distalExpected = 0.211845;
  double primalActual = mBn2->getParentJoint()->getDof(0)->getPosition();
  double distalActual = mBn3->getParentJoint()->getDof(0)->getPosition();

  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  EXPECT_NEAR(primalExpected, primalActual, eps);
  EXPECT_NEAR(distalExpected, distalActual, eps);
  EXPECT_NEAR(primalActual*mimicRatio, distalActual, eps);
}



// ###### TODO 
// TEST_F(BarrettFingerPositionCommandExecutorTest, execute_PrimalInCollision_PrimalStops_DistalContinues)
// {
//   BarrettFingerPositionCommandExecutor executor(mFingerChain, 1, 2);

//   double step = 0.3;
//   double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

//   double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
//   double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
//   double goalPrimal = startPrimal;
//   double goalDistal = goalDistal + step*mimicRatio;

//   executor.execute(step, true);

//   EXPECT_DOUBLE_EQ(mBn2->getParentJoint()->getDof(0)->getPosition(), goalPrimal);
//   EXPECT_DOUBLE_EQ(mBn3->getParentJoint()->getDof(0)->getPosition(), goalDistal);

// }

// TEST_F(BarrettFingerPositionCommandExecutorTest, execute_DistalInCollisionCompletes)
// {

// }

// TEST_F(BarrettFingerPositionCommandExecutorTest, execute_SetPointAbovePrimalUpperLimitStopsAtLimit)
// {
//   BarrettFingerPositionCommandExecutor executor(mFingerChain, 1, 2);

//   double step = 0.7;
//   double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

//   mFingerChain->getDof(1)->setPositionUpperLimit(0.5);

//   double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
//   double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
//   double goalPrimal = 0.5;
//   double goalDistal = goalDistal + step*mimicRatio;

//   executor.execute(step, false);

//   EXPECT_DOUBLE_EQ(goalPrimal, mBn2->getParentJoint()->getDof(0)->getPosition());
//   EXPECT_DOUBLE_EQ(goalDistal, mBn3->getParentJoint()->getDof(0)->getPosition());

// }

// TEST_F(BarrettFingerPositionCommandExecutorTest, execute_SetPointBelowPrimalLowerLimitStopsAtLimit)
// {
//   BarrettFingerPositionCommandExecutor executor(mFingerChain, 1, 2);

//   double step = -0.7;
//   double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

//   mFingerChain->getDof(1)->setPositionLowerLimit(0);

//   double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
//   double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
//   double goalPrimal = 0;
//   double goalDistal = goalDistal + step*mimicRatio;

//   executor.execute(step, false);

//   EXPECT_DOUBLE_EQ(goalPrimal, mBn2->getParentJoint()->getDof(0)->getPosition());
//   EXPECT_DOUBLE_EQ(goalDistal, mBn3->getParentJoint()->getDof(0)->getPosition());

// }

// TEST_F(BarrettFingerPositionCommandExecutorTest, execute_SetPointAboveDistalUpperLimitStopsAtLimit)
// {}

// TEST_F(BarrettFingerPositionCommandExecutorTest, execute_SetPointBelowLowerDistalLimitStopsAtLimit)
// {}