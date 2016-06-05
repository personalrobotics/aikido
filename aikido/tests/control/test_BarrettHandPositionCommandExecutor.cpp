#include <gtest/gtest.h>
#include <aikido/control/BarrettHandPositionCommandExecutor.hpp>
#include <aikido/control/BarrettFingerPositionCommandExecutor.hpp>
#include <aikido/control/BarrettFingerSpreadCommandExecutor.hpp>

#include <dart/dart.h>

#include <chrono>

using aikido::control::BarrettFingerPositionCommandExecutorPtr;
using aikido::control::BarrettFingerSpreadCommandExecutorPtr;

using aikido::control::BarrettHandPositionCommandExecutor;
using aikido::control::BarrettFingerPositionCommandExecutor;
using aikido::control::BarrettFingerSpreadCommandExecutor;

using ::dart::dynamics::Chain;
using ::dart::dynamics::ChainPtr;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;

using namespace dart::dynamics;
using namespace dart::collision;
using namespace dart::gui;
using namespace dart::simulation;


class BarrettHandPositionCommandExecutorTest : public testing::Test
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
    auto mFinger = Skeleton::create("Finger");

    Eigen::Vector3d fingerSize(0.1, 0.1, 0.9);
    std::shared_ptr<BoxShape> fingerShape(new BoxShape(fingerSize));

    // spread joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitX();
    properties1.mName = "Joint1";
    auto mBn1 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties1, 
      BodyNode::Properties(std::string("spread"))).second;
    mBn1->getParentJoint()->setTransformFromParentBodyNode(transform);

    // primal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    auto mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn1, properties2, 
      BodyNode::Properties(std::string("primal"))).second;
    setGeometry(mBn2);

    // distal joint
    RevoluteJoint::Properties properties3;
    properties3.mAxis = Eigen::Vector3d::UnitY();
    properties3.mName = "Joint3";
    properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    auto mBn3 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn2, properties3, 
      BodyNode::Properties(std::string("distal"))).second;
    setGeometry(mBn3);

    Chain::IncludeBoth_t t;
    return Chain::create(mBn1, mBn3, t);
  }

  ChainPtr create2DoFFinger(Eigen::Isometry3d transform
    = Eigen::Isometry3d::Identity())
  {
    auto mFinger = Skeleton::create("Finger");

    Eigen::Vector3d fingerSize(0.1, 0.1, 0.7);
    std::shared_ptr<BoxShape> fingerShape(new BoxShape(fingerSize));

    // primal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    auto mBn1 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
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
    auto mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
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
    // Fingers
    mFingerChains.reserve(3);
    mFingerChains.push_back(create3DoFFinger(
      Eigen::Isometry3d::Identity()));

    Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    transform.linear() = rotation;
    mFingerChains.push_back(create3DoFFinger(transform));

    rotation = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());
    transform.linear() = rotation;
    transform.translation() = Eigen::Vector3d(0, 1.0, 0.0);
    mFingerChains.push_back(create2DoFFinger(transform));
    
    // CollisionDetector
    mCollisionDetector = FCLCollisionDetector::create();
    mCollideWith = mCollisionDetector->createCollisionGroupAsSharedPtr();

    int spreadDof = 0; 
    int primalDof[3] = {1,1,0};
    int distalDof[3] = {2,2,1};

    for(int i = 0 ; i < 3; ++i)
    {
      mPositionExecutors.push_back(std::make_shared<BarrettFingerPositionCommandExecutor>(
        mFingerChains[i], primalDof[i], distalDof[i], mCollisionDetector));
    }

    for(int i = 0; i < 2; ++i)
    {
      mSpreadExecutors.push_back(std::make_shared<BarrettFingerSpreadCommandExecutor>(
        mFingerChains[i], spreadDof, mCollisionDetector));
    }

    mPositions = Eigen::Matrix<double, 4, 1>::Ones();
    mPositions(3) = M_PI/2;
  }

protected:
  std::vector<ChainPtr> mFingerChains;
  CollisionDetectorPtr mCollisionDetector;
  CollisionGroupPtr mCollideWith;
  std::vector<BarrettFingerPositionCommandExecutorPtr> mPositionExecutors;
  std::vector<BarrettFingerSpreadCommandExecutorPtr> mSpreadExecutors;

  Eigen::Matrix<double, 4, 1> mPositions;
  static constexpr double eps = 0.01;

};

TEST_F(BarrettHandPositionCommandExecutorTest, constructor_lessthan3PositionExecutors_throws)
{
  std::vector<BarrettFingerPositionCommandExecutorPtr> positionExecutors; 
  positionExecutors.push_back(mPositionExecutors[0]);
  positionExecutors.push_back(mPositionExecutors[1]);
  
  EXPECT_THROW(BarrettHandPositionCommandExecutor(
    positionExecutors, mSpreadExecutors), std::invalid_argument);
}


TEST_F(BarrettHandPositionCommandExecutorTest, constructor_nullPositionExecutor_throws)
{
  std::vector<BarrettFingerPositionCommandExecutorPtr> positionExecutors; 
  positionExecutors.push_back(mPositionExecutors[0]);
  positionExecutors.push_back(mPositionExecutors[1]);
  positionExecutors.push_back(nullptr);
  
  EXPECT_THROW(BarrettHandPositionCommandExecutor(
    positionExecutors, mSpreadExecutors), std::invalid_argument);
}


TEST_F(BarrettHandPositionCommandExecutorTest, constructor_lessthan2SpreadExecutors_throws)
{
  std::vector<BarrettFingerSpreadCommandExecutorPtr> spreadExecutors; 
  spreadExecutors.push_back(mSpreadExecutors[0]);
  
  EXPECT_THROW(BarrettHandPositionCommandExecutor(
    mPositionExecutors, spreadExecutors), std::invalid_argument);
}


TEST_F(BarrettHandPositionCommandExecutorTest, constructor_nullSpreadExecutor_throws)
{
  std::vector<BarrettFingerSpreadCommandExecutorPtr> spreadExecutors; 
  spreadExecutors.push_back(mSpreadExecutors[0]);
  spreadExecutors.push_back(nullptr);
  
  EXPECT_THROW(BarrettHandPositionCommandExecutor(
    mPositionExecutors, spreadExecutors), std::invalid_argument);
}



TEST_F(BarrettHandPositionCommandExecutorTest, constructor_no_throw)
{
  EXPECT_NO_THROW(BarrettHandPositionCommandExecutor(
    mPositionExecutors, mSpreadExecutors));
}


TEST_F(BarrettHandPositionCommandExecutorTest, execute_WaitOnFuture_CommandExecuted)
{
  // Setup
  BarrettHandPositionCommandExecutor executor(
    mPositionExecutors, mSpreadExecutors);

  double startPrimalDofValues[3];
  double startDistalDofValues[3];
  double startSpread = mFingerChains[0]->getDof(0)->getPosition();
  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  for(int i = 0 ; i < 2; ++i)
  {
    startPrimalDofValues[i] = mFingerChains[i]->getDof(1)->getPosition();
  }
  startPrimalDofValues[2] = mFingerChains[2]->getDof(0)->getPosition();

  for(int i = 0 ; i < 2; ++i)
  {
    startDistalDofValues[i] = mFingerChains[i]->getDof(2)->getPosition();
  }
  startDistalDofValues[2] = mFingerChains[2]->getDof(1)->getPosition();

  // Execute trajectory
  auto future = executor.execute(mPositions, mCollideWith);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  // Validate dof values
  for(int i = 0; i < 2; ++i)
  {
    double primal = mFingerChains[i]->getDof(1)->getPosition();
    double distal = mFingerChains[i]->getDof(2)->getPosition();

    EXPECT_NEAR(mPositions(i), primal, eps);
    EXPECT_NEAR(mPositions(i)*mimicRatio, distal, eps);

    double spread = mFingerChains[i]->getDof(0)->getPosition();
    EXPECT_NEAR(spread, mPositions(3), eps);
  }

  double primal = mFingerChains[2]->getDof(0)->getPosition();
  double distal = mFingerChains[2]->getDof(1)->getPosition();

  EXPECT_NEAR(mPositions(2), primal, eps);
  EXPECT_NEAR(mPositions(2)*mimicRatio, distal, eps);
}

TEST_F(BarrettHandPositionCommandExecutorTest, execute_CommandIsAlreadyRunning_Throws)
{
  BarrettHandPositionCommandExecutor executor(
    mPositionExecutors, mSpreadExecutors);

  // Execute trajectory
  auto future = executor.execute(mPositions, mCollideWith);
  EXPECT_THROW(executor.execute(mPositions, mCollideWith),
    std::runtime_error);
}

TEST_F(BarrettHandPositionCommandExecutorTest, execute_PrevCommandFinished_DoesNotThrow)
{
  BarrettHandPositionCommandExecutor executor(
    mPositionExecutors, mSpreadExecutors);

  // Execute trajectory
  auto future = executor.execute(mPositions, mCollideWith);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  EXPECT_NO_THROW(executor.execute(mPositions, mCollideWith));
}

TEST_F(BarrettHandPositionCommandExecutorTest, execute_PrimalStopsAtCollsionDistalContinues)
{
  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
  transform.translation() = Eigen::Vector3d(0.3, 0, 0.3);
  auto ball = createBall(transform, mCollisionDetector);
  auto collideWith = mCollisionDetector->createCollisionGroup(ball);

  Eigen::VectorXd position(Eigen::VectorXd::Zero(4));
  double goal = M_PI;
  position(0) = goal;
  
  BarrettHandPositionCommandExecutor executor(
    mPositionExecutors, mSpreadExecutors);

  auto future = executor.execute(position, std::move(collideWith));

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  double primal = mFingerChains[0]->getDof(1)->getPosition();
  double distal = mFingerChains[0]->getDof(2)->getPosition();
  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  // Values made by visual inspection
  EXPECT_NEAR(0.56548, primal, eps);
  EXPECT_NEAR(goal*mimicRatio, distal, eps);
}

TEST_F(BarrettHandPositionCommandExecutorTest, execute_DistalStopsAtCollsionPromalAlsoStops)
{
  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
  transform.translation() = Eigen::Vector3d(1.3, 0, 1.3);
  auto ball = createBall(transform, mCollisionDetector);
  auto collideWith = mCollisionDetector->createCollisionGroup(ball);

  Eigen::VectorXd position(Eigen::VectorXd::Zero(4));
  double goal = M_PI/4;
  position(0) = goal;

  
  BarrettHandPositionCommandExecutor executor(
    mPositionExecutors, mSpreadExecutors);
  auto future = executor.execute(position, std::move(collideWith));
  
  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  double primal = mFingerChains[0]->getDof(1)->getPosition();
  double distal = mFingerChains[0]->getDof(2)->getPosition();

  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  // Values made by visual inspection
  EXPECT_NEAR(0.211845, distal, eps);
  EXPECT_NEAR(0.636173, primal, eps);
  EXPECT_NEAR(primal*mimicRatio, distal, eps);
}

