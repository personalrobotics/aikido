#include <gtest/gtest.h>
#include <aikido/control/BarrettFingerPositionCommandExecutor.hpp>
#include <dart/dart.hpp>
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

static BodyNode::Properties create_BodyNodeProperties(const std::string& _name)
{
  BodyNode::Properties properties;
  properties.mName = _name;
  return properties;
}

class BarrettFingerPositionCommandExecutorTest : public testing::Test
{
public:

  void setGeometry(const BodyNodePtr& bn)
  {
    // Create a BoxShape to be used for both visualization and collision checking
    Eigen::Vector3d fingerSize(0.1, 0.1, 0.8);
    std::shared_ptr<BoxShape> fingerShape(new BoxShape(fingerSize));

    // Create a shpae node for visualization and collision checking
    auto shapeNode
        = bn->createShapeNodeWith<VisualAspect,
          CollisionAspect, DynamicsAspect>(fingerShape);

    // Set the location of the shape node
    Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
    Eigen::Vector3d center = Eigen::Vector3d(0, 0, 1.0 / 2.0);
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
      nullptr, properties1, create_BodyNodeProperties("spread")).second;
    mBn1->getParentJoint()->setTransformFromParentBodyNode(transform);

    // proximal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn1, properties2, create_BodyNodeProperties("proximal")).second;
    mBn2->getParentJoint()->setPositionUpperLimit(0, M_PI);
    mBn2->getParentJoint()->setPositionLowerLimit(0, -M_PI);    
    setGeometry(mBn2);

    // distal joint
    RevoluteJoint::Properties properties3;
    properties3.mAxis = Eigen::Vector3d::UnitY();
    properties3.mName = "Joint3";
    properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    mBn3 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn2, properties3, create_BodyNodeProperties("distal")).second;
    mBn3->getParentJoint()->setPositionUpperLimit(0, M_PI);
    mBn3->getParentJoint()->setPositionLowerLimit(0, -M_PI);    
    setGeometry(mBn3);

    return Chain::create(mBn1, mBn3, Chain::IncludeBoth);
  }

  ChainPtr create2DoFFinger(Eigen::Isometry3d transform
    = Eigen::Isometry3d::Identity())
  {
    mFinger = Skeleton::create("Finger");

    Eigen::Vector3d fingerSize(0.1, 0.1, 0.7);
    std::shared_ptr<BoxShape> fingerShape(new BoxShape(fingerSize));

    // proximal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    mBn1 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties2, create_BodyNodeProperties("proximal")).second;
    mBn1->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      fingerShape);
    mBn1->getParentJoint()->setTransformFromParentBodyNode(transform);
    mBn1->getParentJoint()->setPositionUpperLimit(0, M_PI);
    mBn1->getParentJoint()->setPositionLowerLimit(0, -M_PI);    

    setGeometry(mBn1);

    // distal joint
    RevoluteJoint::Properties properties3;
    properties3.mAxis = Eigen::Vector3d::UnitY();
    properties3.mName = "Joint3";
    properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn1, properties3, create_BodyNodeProperties("distal")).second;
    setGeometry(mBn2);
    mBn2->getParentJoint()->setPositionUpperLimit(0, M_PI);
    mBn2->getParentJoint()->setPositionLowerLimit(0, -M_PI);    

    return Chain::create(mBn1, mBn2, Chain::IncludeBoth);
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
      ballBody->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
        ballShape);
    ballBody->getParentJoint()->setTransformFromParentBodyNode(
      transform);

    return ballBody;
  }

  virtual void SetUp()
  {
    mFingerChain = create3DoFFinger(Eigen::Isometry3d::Identity());

    // CollisionDetector
    mCollisionDetector = FCLCollisionDetector::create();
    mCollideWith = mCollisionDetector->createCollisionGroupAsSharedPtr();
 
    mPosition = 1;
    mProximalDof = 1;
    mDistalDof = 2;
  }

protected:
  SkeletonPtr mFinger;
  ChainPtr mFingerChain;
  BodyNodePtr mBn1, mBn2, mBn3;

  int mProximalDof, mDistalDof;

  std::chrono::milliseconds mDuration;
  CollisionDetectorPtr mCollisionDetector;
  CollisionGroupPtr mCollideWith;

  double mPosition;
  static constexpr double eps = 1e-2;
};


TEST_F(BarrettFingerPositionCommandExecutorTest, constructor_NullChain_Throws)
{
  EXPECT_THROW(BarrettFingerPositionCommandExecutor executor(
    nullptr, mProximalDof, mDistalDof, mCollisionDetector),
    std::invalid_argument);
}

TEST_F(BarrettFingerPositionCommandExecutorTest, constructor)
{
  EXPECT_NO_THROW(BarrettFingerPositionCommandExecutor executor(
    mFingerChain, mProximalDof, mDistalDof, mCollisionDetector));
}

TEST_F(BarrettFingerPositionCommandExecutorTest, constructor_Nonexistingproximal_throws)
{
  int proximalDof = 3;
  EXPECT_THROW(BarrettFingerPositionCommandExecutor executor(
    mFingerChain, proximalDof, mDistalDof, mCollisionDetector),
    std::invalid_argument);
}

TEST_F(BarrettFingerPositionCommandExecutorTest, constructor_NonexistingDistal_throws)
{
  int distalDof = 3;
  EXPECT_THROW(BarrettFingerPositionCommandExecutor executor(
    mFingerChain, mProximalDof, distalDof, mCollisionDetector),
    std::invalid_argument);
}

TEST_F(BarrettFingerPositionCommandExecutorTest, execute_WaitOnFuture_CommandExecuted)
{
  BarrettFingerPositionCommandExecutor executor(
    mFingerChain, mProximalDof, mDistalDof, mCollisionDetector);

  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  double startproximal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalProximal = mPosition;
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

  EXPECT_NEAR(goalProximal, mBn2->getParentJoint()->getDof(0)->getPosition(), eps);
  EXPECT_NEAR(goalDistal, mBn3->getParentJoint()->getDof(0)->getPosition(), eps);
}

TEST_F(BarrettFingerPositionCommandExecutorTest, 
  execute_proximalStopsAtCollsionDistalContinuesUntilCollision)
{
  // Collision obstacle
  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
  transform.translation() = Eigen::Vector3d(0.3, 0, 0.3);
  auto ball = createBall(transform, mCollisionDetector);
  auto collideWith = mCollisionDetector->createCollisionGroup(ball);

  double goal = M_PI;

  BarrettFingerPositionCommandExecutor executor(
    mFingerChain, mProximalDof, mDistalDof, mCollisionDetector);

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

  double proximalExpected = 0.56548; // Value made by visual inspection
  double proximalActual = mBn2->getParentJoint()->getDof(0)->getPosition();
  double distalExpected = 2.81718;
  double distalActual = mBn3->getParentJoint()->getDof(0)->getPosition();

  EXPECT_NEAR(proximalExpected, proximalActual, eps);
  EXPECT_NEAR(distalExpected, distalActual, eps);
}


TEST_F(BarrettFingerPositionCommandExecutorTest, execute_DistalStopsAtCollsionProximalAlsoStops)
{
  // Collision object
  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
  transform.translation() = Eigen::Vector3d(1.3, 0, 1.3);
  auto ball = createBall(transform, mCollisionDetector);
  auto collideWith = mCollisionDetector->createCollisionGroup(ball);

  // Executor
  BarrettFingerPositionCommandExecutor executor(
    mFingerChain, mProximalDof, mDistalDof, mCollisionDetector);
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

  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  // Values made by visual inspection
  double distalExpected = 0.21312;
  double proximalExpected = distalExpected*1.0/mimicRatio;
  double proximalActual = mBn2->getParentJoint()->getDof(0)->getPosition();
  double distalActual = mBn3->getParentJoint()->getDof(0)->getPosition();

  EXPECT_NEAR(proximalExpected, proximalActual, eps);
  EXPECT_NEAR(distalExpected, distalActual, eps);
}
