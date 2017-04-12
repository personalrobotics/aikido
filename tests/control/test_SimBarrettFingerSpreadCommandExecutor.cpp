#include <gtest/gtest.h>
#include <aikido/control/SimBarrettFingerSpreadCommandExecutor.hpp>
#include <dart/dart.hpp>

#include <chrono>

using aikido::control::SimBarrettFingerSpreadCommandExecutor;
using ::dart::dynamics::Chain;
using ::dart::dynamics::ChainPtr;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;

using namespace dart::dynamics;
using namespace dart::collision;
using namespace dart::simulation;

static BodyNode::Properties create_BodyNodeProperties(const std::string& _name)
{
  BodyNode::Properties properties;
  properties.mName = _name;
  return properties;
}

class SimBarrettFingerSpreadCommandExecutorTest : public testing::Test
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

  ChainPtr create3DoFFinger(
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity(),
    Eigen::Vector3d spreadAxis = Eigen::Vector3d::UnitX())
  {
    auto finger = Skeleton::create("Finger");

    // spread joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = spreadAxis;
    properties1.mName = "Joint1";
    auto bn1 = finger->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties1, create_BodyNodeProperties("spread")).second;
    bn1->getParentJoint()->setTransformFromParentBodyNode(transform);
    bn1->getParentJoint()->setPositionUpperLimit(0, M_PI);
    bn1->getParentJoint()->setPositionLowerLimit(0, 0);

    // proximal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    auto bn2 = finger->createJointAndBodyNodePair<RevoluteJoint>(
      bn1, properties2, create_BodyNodeProperties("proximal")).second;
    bn2->getParentJoint()->setPositionUpperLimit(0, M_PI);
    bn2->getParentJoint()->setPositionLowerLimit(0, -M_PI);
    setGeometry(bn2);

    // distal joint
    RevoluteJoint::Properties properties3;
    properties3.mAxis = Eigen::Vector3d::UnitY();
    properties3.mName = "Joint3";
    properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    auto bn3 = finger->createJointAndBodyNodePair<RevoluteJoint>(
      bn2, properties3, create_BodyNodeProperties("distal")).second;
    bn3->getParentJoint()->setPositionUpperLimit(0, M_PI);
    bn3->getParentJoint()->setPositionLowerLimit(0, -M_PI);
    setGeometry(bn3);

    return Chain::create(bn1, bn3, Chain::IncludeBoth);
  }

  /// \param transform pose of ball
  /// \param collisionDetector CollisionDetector to create CollisionGroup with
  BodyNodePtr createBall(Eigen::Isometry3d transform,
    ::dart::collision::CollisionDetectorPtr /*collisionDetector*/)
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
    // Create 2 fingers
    mFingerChains[0] = create3DoFFinger(Eigen::Isometry3d::Identity());

    Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    transform.linear() = rotation;
    mFingerChains[1] = create3DoFFinger(transform, -Eigen::Vector3d::UnitX());

    // CollisionDetector
    mCollisionDetector = FCLCollisionDetector::create();
    mCollideWith = mCollisionDetector->createCollisionGroupAsSharedPtr();

    mPosition = 1;
    mSpreadDof = 0;
  }

protected:
  SkeletonPtr mFinger;
  std::array<ChainPtr, 2> mFingerChains;

  CollisionDetectorPtr mCollisionDetector;
  CollisionGroupPtr mCollideWith;

  double mPosition;
  int mSpreadDof;
  static constexpr double eps = 2e-1;
};

TEST_F(SimBarrettFingerSpreadCommandExecutorTest, constructor)
{
  EXPECT_NO_THROW(SimBarrettFingerSpreadCommandExecutor(
    mFingerChains, mSpreadDof, mCollisionDetector, mCollideWith));
}

TEST_F(SimBarrettFingerSpreadCommandExecutorTest, constructor_NonexistingSpread_throws)
{
  int spreadDof = 3;
  EXPECT_THROW(SimBarrettFingerSpreadCommandExecutor(
    mFingerChains, spreadDof, mCollisionDetector, mCollideWith),
    std::invalid_argument);
}


TEST_F(SimBarrettFingerSpreadCommandExecutorTest, constructor_NullCollideWith_throws)
{
  int spreadDof = 3;
  EXPECT_THROW(SimBarrettFingerSpreadCommandExecutor(
    mFingerChains, spreadDof, mCollisionDetector, nullptr),
    std::invalid_argument);
}

TEST_F(SimBarrettFingerSpreadCommandExecutorTest, execute_WaitOnFuture_CommandExecuted)
{
  SimBarrettFingerSpreadCommandExecutor executor(
    mFingerChains, mSpreadDof, mCollisionDetector, mCollideWith);

  auto future = executor.execute(mPosition);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  for (auto finger : mFingerChains)
  {
    auto dof = finger->getBodyNode(0)->getParentJoint()->getDof(mSpreadDof);
    double spread = dof->getPosition();
    EXPECT_NEAR(mPosition, spread, eps);
  }
}

TEST_F(SimBarrettFingerSpreadCommandExecutorTest,
  execute_BothFingersStopAtProximalCollision)
{

  // Collision object
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, -0.8, 1.5);
  auto ball = createBall(transform, mCollisionDetector);
  mCollideWith = mCollisionDetector->createCollisionGroup(ball);

  SimBarrettFingerSpreadCommandExecutor executor(
    mFingerChains, mSpreadDof, mCollisionDetector, mCollideWith);

  double goal = 1.0;
  auto future = executor.execute(goal);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  for (auto finger : mFingerChains)
  {
    double spread = finger->getBodyNode(0)->getParentJoint()
                    ->getDof(mSpreadDof)->getPosition();
    double expected = 0.4;
    EXPECT_NEAR(expected, spread, eps);
  }
}

TEST_F(SimBarrettFingerSpreadCommandExecutorTest,
  execute_BothFingersStopAtDistalCollision)
{

  // Collision object
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0, -0.4, 0.7);
  auto ball = createBall(transform, mCollisionDetector);
  mCollideWith = mCollisionDetector->createCollisionGroup(ball);

  SimBarrettFingerSpreadCommandExecutor executor(
    mFingerChains, mSpreadDof, mCollisionDetector, mCollideWith);

  double goal = 1.0;
  auto future = executor.execute(goal);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  for (auto finger : mFingerChains)
  {
    double spread = finger->getBodyNode(0)->getParentJoint()
                    ->getDof(mSpreadDof)->getPosition();
    double expected = 0.44;
    EXPECT_NEAR(expected, spread, eps);
  }
}

TEST_F(SimBarrettFingerSpreadCommandExecutorTest,
  execute_ThrowsWhenFingerSpreadValuesDiffer)
{
  SimBarrettFingerSpreadCommandExecutor executor(
    mFingerChains, mSpreadDof, mCollisionDetector, mCollideWith);

  mFingerChains[0]->getBodyNode(0)->getParentJoint()->getDof(0)->setPosition(0);
  mFingerChains[1]->getBodyNode(0)->getParentJoint()->getDof(0)->setPosition(M_PI/4);

  double goal = M_PI*0.5;
  auto future = executor.execute(goal);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  EXPECT_THROW(future.get(), std::runtime_error);
}

TEST_F(SimBarrettFingerSpreadCommandExecutorTest,
  execute_GoalAboveUpperLimitStopsAtUpperJointLimit)
{
  SimBarrettFingerSpreadCommandExecutor executor(
    mFingerChains, mSpreadDof, mCollisionDetector, mCollideWith);

  double goal = M_PI*1.5;
  auto future = executor.execute(goal);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  for (auto finger : mFingerChains)
  {
    double spread = finger->getBodyNode(0)->getParentJoint()
                    ->getDof(mSpreadDof)->getPosition();
    double expected = finger->getBodyNode(0)->getParentJoint()
                    ->getDof(mSpreadDof)->getPositionUpperLimit();
    EXPECT_NEAR(expected, spread, eps);
  }
}


TEST_F(SimBarrettFingerSpreadCommandExecutorTest,
  execute_GoalBelowLowerLimitStopsAtLowerJointLimit)
{
  SimBarrettFingerSpreadCommandExecutor executor(
    mFingerChains, mSpreadDof, mCollisionDetector, mCollideWith);

  double goal = -M_PI*0.5;
  auto future = executor.execute(goal);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  for (auto finger : mFingerChains)
  {
    double spread = finger->getBodyNode(0)->getParentJoint()
                    ->getDof(mSpreadDof)->getPosition();
    double expected = finger->getBodyNode(0)->getParentJoint()
                    ->getDof(mSpreadDof)->getPositionLowerLimit();
    EXPECT_NEAR(expected, spread, eps);
  }
}

