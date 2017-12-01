#include <gtest/gtest.h>
#include <tuple>
#include <dart/dart.hpp>
#include <aikido/constraint/CollisionFree.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include "../../constraint/MockConstraints.hpp"

using std::shared_ptr;
using std::make_shared;

class VectorFieldPlannerTest : public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
  using CollisionFree = aikido::constraint::CollisionFree;
  using DistanceMetric = aikido::distance::DistanceMetric;
  using MetaSkeletonStateSpacePtr
      = aikido::statespace::dart::MetaSkeletonStateSpacePtr;
  using MetaSkeletonStateSpace
      = aikido::statespace::dart::MetaSkeletonStateSpace;
  using SO2 = aikido::statespace::SO2;
  using SE3 = aikido::statespace::SE3;
  using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
  using ScopedState = MetaSkeletonStateSpace::ScopedState;

  using BodyNode = dart::dynamics::BodyNode;
  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using Joint = dart::dynamics::Joint;
  using Skeleton = dart::dynamics::Skeleton;
  using Shape = dart::dynamics::Shape;
  using BoxShape = dart::dynamics::BoxShape;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  using PrismaticJoint = dart::dynamics::PrismaticJoint;
  using RevoluteJoint = dart::dynamics::RevoluteJoint;
  using WeldJoint = dart::dynamics::WeldJoint;
  using BallJoint = dart::dynamics::BallJoint;
  using VisualAspect = dart::dynamics::VisualAspect;
  using CollisionAspect = dart::dynamics::CollisionAspect;
  using DynamicsAspect = dart::dynamics::DynamicsAspect;
  using constantsd = dart::math::constantsd;
  using Vector3d = Eigen::Vector3d;

  enum TypeOfDOF
  {
    DOF_X,
    DOF_Y,
    DOF_Z,
    DOF_ROLL,
    DOF_PITCH,
    DOF_YAW,
    BALL
  };

  VectorFieldPlannerTest() : mErrorTolerance(1e-3)
  {
    SkeletonPtr skel = createNLinkRobot();
    mNumDof = skel->getNumDofs();

    mUpperPositionLimits = Eigen::VectorXd::Constant(mNumDof, constantsd::pi());
    mLowerPositionLimits
        = Eigen::VectorXd::Constant(mNumDof, -constantsd::pi());
    mUpperVelocityLimits = Eigen::VectorXd::Constant(mNumDof, 4.0);
    mLowerVelocityLimits = Eigen::VectorXd::Constant(mNumDof, -4.0);

    skel->setPositionUpperLimits(mUpperPositionLimits);
    skel->setPositionLowerLimits(mLowerPositionLimits);
    skel->setVelocityUpperLimits(mUpperVelocityLimits);
    skel->setVelocityLowerLimits(mLowerVelocityLimits);
    mWorkspaceStateSpace = std::make_shared<SE3>();
    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(skel);
    mBodynode = mStateSpace->getMetaSkeleton()->getBodyNodes().back();

    mStartConfig = Eigen::VectorXd::Zero(mNumDof);
    mStartConfig << 2.13746, -0.663612, 1.77876, 1.87515, 2.58646, -1.90034,
        -1.03533;

    mGoalConfig = Eigen::VectorXd(mNumDof);
    mPassingConstraint = std::make_shared<PassingConstraint>(mStateSpace);
  }

  std::pair<Joint*, BodyNode*> add1DofJoint(
      SkeletonPtr _skel,
      BodyNode* _parent,
      const BodyNode::Properties& _node,
      const std::string& _name,
      double _val,
      double _min,
      double _max,
      int _type)
  {
    dart::dynamics::GenericJoint<dart::math::R1Space>::Properties properties(
        _name);
    properties.mPositionLowerLimits[0] = _min;
    properties.mPositionUpperLimits[0] = _max;
    std::pair<Joint*, BodyNode*> newComponent;
    if (DOF_X == _type)
      newComponent = _skel->createJointAndBodyNodePair<PrismaticJoint>(
          _parent,
          PrismaticJoint::Properties(
              properties, Eigen::Vector3d(1.0, 0.0, 0.0)),
          _node);
    else if (DOF_Y == _type)
      newComponent = _skel->createJointAndBodyNodePair<PrismaticJoint>(
          _parent,
          PrismaticJoint::Properties(
              properties, Eigen::Vector3d(0.0, 1.0, 0.0)),
          _node);
    else if (DOF_Z == _type)
      newComponent = _skel->createJointAndBodyNodePair<PrismaticJoint>(
          _parent,
          PrismaticJoint::Properties(
              properties, Eigen::Vector3d(0.0, 0.0, 1.0)),
          _node);
    else if (DOF_YAW == _type)
      newComponent = _skel->createJointAndBodyNodePair<RevoluteJoint>(
          _parent,
          RevoluteJoint::Properties(properties, Eigen::Vector3d(0.0, 0.0, 1.0)),
          _node);
    else if (DOF_PITCH == _type)
      newComponent = _skel->createJointAndBodyNodePair<RevoluteJoint>(
          _parent,
          RevoluteJoint::Properties(properties, Eigen::Vector3d(0.0, 1.0, 0.0)),
          _node);
    else if (DOF_ROLL == _type)
      newComponent = _skel->createJointAndBodyNodePair<RevoluteJoint>(
          _parent,
          RevoluteJoint::Properties(properties, Eigen::Vector3d(1.0, 0.0, 0.0)),
          _node);

    newComponent.first->setPosition(0, _val);
    return newComponent;
  }

  //==============================================================================
  /// Creates a N link manipulator with the given dimensions where each joint is
  /// the specified type
  SkeletonPtr createNLinkRobot()
  {
    Vector3d dim(0.1, 0.1, 0.2);

    SkeletonPtr robot = Skeleton::create();
    // robot->disableSelfCollision();

    // Create the first link, the joint with the ground and its shape
    BodyNode::Properties node(BodyNode::AspectProperties("link0"));
    //  node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
    std::shared_ptr<Shape> shape(new BoxShape(dim));

    std::pair<Joint*, BodyNode*> pair1;
    pair1 = add1DofJoint(
        robot,
        nullptr,
        node,
        "joint0",
        0.0,
        -constantsd::pi(),
        constantsd::pi(),
        DOF_ROLL);

    Joint* joint = pair1.first;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // T.translation() = Eigen::Vector3d(0.0, 0.0, -0.5*dim(2));
    // joint->setTransformFromParentBodyNode(T);
    T.translation() = Eigen::Vector3d(0.0, 0.0, 0.5 * dim(2));
    joint->setTransformFromChildBodyNode(T);
    // joint->setDampingCoefficient(0, 0.01);

    auto currentNode = pair1.second;
    currentNode
        ->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
            shape);

    BodyNode* parentNode = currentNode;

    // Create links iteratively
    for (int i = 1; i < 7; ++i)
    {
      std::ostringstream ssLink;
      std::ostringstream ssJoint;
      ssLink << "link" << i;
      ssJoint << "joint" << i;

      node = BodyNode::Properties(BodyNode::AspectProperties(ssLink.str()));
      //    node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
      shape = std::shared_ptr<Shape>(new BoxShape(dim));

      std::pair<Joint*, BodyNode*> newPair;

      TypeOfDOF type;
      if (i % 2 == 1)
        type = DOF_PITCH;
      else
        type = DOF_ROLL;
      newPair = add1DofJoint(
          robot,
          parentNode,
          node,
          ssJoint.str(),
          0.0,
          -constantsd::pi(),
          constantsd::pi(),
          type);

      Joint* joint = newPair.first;
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.translation() = Eigen::Vector3d(0.0, 0.0, -0.5 * dim(2));
      joint->setTransformFromParentBodyNode(T);
      T.translation() = Eigen::Vector3d(0.0, 0.0, 0.5 * dim(2));
      joint->setTransformFromChildBodyNode(T);
      // joint->setDampingCoefficient(0, 0.01);

      auto current_node = newPair.second;
      current_node
          ->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
              shape);

      parentNode = current_node;
    }

    // If finished, initialize the skeleton
    // addEndEffector(robot, parentNode, dim);

    return robot;
  }

  // Parameters
  double mErrorTolerance;

  // DART setup
  SkeletonPtr mSkel;
  std::vector<std::pair<JointPtr, BodyNodePtr>> mJnBn;
  MetaSkeletonStateSpacePtr mStateSpace;
  std::shared_ptr<SE3> mWorkspaceStateSpace;
  BodyNodePtr mBodynode;
  std::size_t mNumDof;
  double mLinearVelocity;

  Eigen::VectorXd mUpperPositionLimits;
  Eigen::VectorXd mLowerPositionLimits;
  Eigen::VectorXd mUpperVelocityLimits;
  Eigen::VectorXd mLowerVelocityLimits;

  // Arguments for planner
  Eigen::VectorXd mStartConfig;
  Eigen::VectorXd mGoalConfig;
  shared_ptr<PassingConstraint> mPassingConstraint;
};

TEST_F(VectorFieldPlannerTest, ComputeJointVelocityFromTwistTest)
{
  using aikido::planner::vectorfield::computeJointVelocityFromTwist;

  Eigen::VectorXd jointVelocityUpperLimits
      = mStateSpace->getMetaSkeleton()->getVelocityUpperLimits();
  Eigen::VectorXd jointVelocityLowerLimits
      = mStateSpace->getMetaSkeleton()->getVelocityLowerLimits();

  double maxStepSize = 0.01;
  double padding = 2e-3;
  double optimizationTolerance = 1; // 1e-3;// 1e-10;

  // Linear only
  Eigen::Vector6d desiredTwist1 = Eigen::Vector6d::Zero();
  desiredTwist1.tail<3>() << 1.0, 1.0, 1.0;

  Eigen::VectorXd qd = Eigen::VectorXd::Zero(mNumDof);
  EXPECT_TRUE(
      computeJointVelocityFromTwist(
          qd,
          desiredTwist1,
          mStateSpace,
          mBodynode,
          padding,
          jointVelocityLowerLimits,
          jointVelocityUpperLimits,
          true,
          maxStepSize,
          optimizationTolerance));

  // Angular only
  Eigen::Vector6d desiredTwist2 = Eigen::Vector6d::Zero();
  desiredTwist2.head<3>() << 1.0, 1.0, 1.0;

  EXPECT_TRUE(
      computeJointVelocityFromTwist(
          qd,
          desiredTwist2,
          mStateSpace,
          mBodynode,
          padding,
          jointVelocityLowerLimits,
          jointVelocityUpperLimits,
          true,
          maxStepSize,
          optimizationTolerance));

  // Both linear and angular
  Eigen::Vector6d desiredTwist3 = Eigen::Vector6d::Zero();
  desiredTwist3.head<3>() << 1.0, 1.0, 1.0;
  desiredTwist3.tail<3>() << 1.0, 1.0, 1.0;

  EXPECT_TRUE(
      computeJointVelocityFromTwist(
          qd,
          desiredTwist2,
          mStateSpace,
          mBodynode,
          padding,
          jointVelocityLowerLimits,
          jointVelocityUpperLimits,
          true,
          maxStepSize,
          optimizationTolerance));
}

TEST_F(VectorFieldPlannerTest, TimeTrajectoryByGeodesicUnitTiming)
{
  using aikido::statespace::Interpolator;
  using dart::common::make_unique;
  using std::make_shared;
  // Create SE3 trajectory
  std::shared_ptr<Interpolator> interpolator
      = make_shared<aikido::statespace::GeodesicInterpolator>(
          mWorkspaceStateSpace);
  auto workpath = std::make_shared<aikido::trajectory::Interpolated>(
      mWorkspaceStateSpace, interpolator);

  Eigen::Isometry3d T0, T1, T2;
  T0.matrix() = Eigen::Matrix4d::Identity();
  T1.matrix() = Eigen::Matrix4d::Identity();
  T1.linear()
      *= dart::math::eulerXYXToMatrix(Eigen::Vector3d(45 * M_PI / 180, 0, 0));
  T1.translation() = Eigen::Vector3d(1, 0, 1);
  T2.matrix() = Eigen::Matrix4d::Identity();
  T2.translation() = Eigen::Vector3d(1, 1, 1);
  auto t0State = mWorkspaceStateSpace->createState();
  aikido::statespace::SE3::State* t0SE3state
      = static_cast<aikido::statespace::SE3::State*>(t0State);
  t0SE3state->setIsometry(T0);
  auto t1State = mWorkspaceStateSpace->createState();
  aikido::statespace::SE3::State* t1SE3state
      = static_cast<aikido::statespace::SE3::State*>(t1State);
  t1SE3state->setIsometry(T1);
  auto t2State = mWorkspaceStateSpace->createState();
  aikido::statespace::SE3::State* t2SE3state
      = static_cast<aikido::statespace::SE3::State*>(t2State);
  t2SE3state->setIsometry(T2);

  workpath->addWaypoint(0, t0State);
  workpath->addWaypoint(1, t1State);
  workpath->addWaypoint(2, t2State);

  double tolerance = 1e-2;

  auto timedTraj
      = aikido::planner::vectorfield::timeTrajectoryByGeodesicUnitTiming(
          workpath.get(), mWorkspaceStateSpace, 0.0);

  double duration = timedTraj->getDuration();
  double expectedDuration = 2.41421356;
  EXPECT_NEAR(duration, expectedDuration, tolerance)
      << "The trajectory duration " << duration
      << " doesnt match the expected duration " << expectedDuration;

  auto timedTraj2
      = aikido::planner::vectorfield::timeTrajectoryByGeodesicUnitTiming(
          workpath.get(), mWorkspaceStateSpace, 1.0);

  duration = timedTraj2->getDuration();
  expectedDuration = 2.88915342;
  EXPECT_NEAR(duration, expectedDuration, tolerance)
      << "The trajectory duration " << duration
      << " doesnt match the expected duration " << expectedDuration;
}

TEST_F(
    VectorFieldPlannerTest, GetMinDistanceBetweenTransformAndWorkspaceTrajTest)
{
  using aikido::statespace::Interpolator;
  using dart::common::make_unique;
  using std::make_shared;
  // Create SE3 trajectory
  std::shared_ptr<Interpolator> interpolator
      = make_shared<aikido::statespace::GeodesicInterpolator>(
          mWorkspaceStateSpace);
  auto workpath = std::make_shared<aikido::trajectory::Interpolated>(
      mWorkspaceStateSpace, interpolator);

  Eigen::Isometry3d T0, T1, T2;
  T0 = Eigen::Isometry3d::Identity();
  T1 = Eigen::Isometry3d::Identity();
  T1.translation() = Eigen::Vector3d(5, 0, 0);
  T2 = Eigen::Isometry3d::Identity();
  T2.translation() = Eigen::Vector3d(0.5, 2, 0.0);
  auto t0State = mWorkspaceStateSpace->createState();
  aikido::statespace::SE3::State* t0SE3state
      = static_cast<aikido::statespace::SE3::State*>(t0State);
  mWorkspaceStateSpace->setIsometry(t0SE3state, T0);
  auto t1State = mWorkspaceStateSpace->createState();
  aikido::statespace::SE3::State* t1SE3state
      = static_cast<aikido::statespace::SE3::State*>(t1State);
  mWorkspaceStateSpace->setIsometry(t1SE3state, T1);

  workpath->addWaypoint(0, t0State);
  workpath->addWaypoint(1, t1State);

  auto timedWorkpath = aikido::planner::vectorfield::timeTrajectoryByGeodesicUnitTiming(
      workpath.get(),
      mWorkspaceStateSpace);

  std::cout << "Duration " << timedWorkpath->getDuration() << std::endl;

  Eigen::Isometry3d test0, test1, test2;
  auto state = mWorkspaceStateSpace->createState();
  timedWorkpath->evaluate(timedWorkpath->getStartTime(), state);
  test0 = mWorkspaceStateSpace->getIsometry(state);
  timedWorkpath->evaluate(timedWorkpath->getEndTime(), state);
  test1 = mWorkspaceStateSpace->getIsometry(state);
  timedWorkpath->evaluate((timedWorkpath->getStartTime() + timedWorkpath->getEndTime())/2., state);
  test2 = mWorkspaceStateSpace->getIsometry(state);

  EXPECT_TRUE(T0.matrix().isApprox(test0.matrix()));
  EXPECT_TRUE(T1.matrix().isApprox(test1.matrix()));

  double minDist = 0.0;
  double tLoc = 0.0;
  Eigen::Isometry3d transLoc;
  aikido::planner::vectorfield::getMinDistanceBetweenTransformAndWorkspaceTraj(
      T2, timedWorkpath.get(), mWorkspaceStateSpace, 0.01, minDist, tLoc, transLoc);

  // The position on the workspace trajectory is half-way along, which is where
  // t = 0.5
  double expectedTLoc = 0.5;
  Eigen::Isometry3d expectedTransLoc;
  expectedTransLoc.matrix() = Eigen::Matrix4d::Identity();
  expectedTransLoc.translation() = Eigen::Vector3d(0.5, 0, 0);

  double tolerance = 0.01;
  EXPECT_NEAR(tLoc, expectedTLoc, tolerance);

  EXPECT_TRUE(transLoc.isApprox(expectedTransLoc, tolerance));
}

TEST_F(VectorFieldPlannerTest, PlanToEndEffectorOffsetTest)
{
  Eigen::Vector3d direction;
  direction << 1., 1., 0.;
  direction.normalize();
  double distance = 0.4;
  double maxDistance = aikido::planner::vectorfield::
      MoveEndEffectorOffsetVectorField::InvalidMaxDistance;

  mStateSpace->getMetaSkeleton()->setPositions(mStartConfig);
  auto startState = mStateSpace->createState();
  mStateSpace->convertPositionsToState(mStartConfig, startState);
  Eigen::Isometry3d startTrans = mBodynode->getTransform();
  Eigen::VectorXd startVec = startTrans.translation();

  double positionTolerance = 0.01;
  double angularTolerance = 0.15;
  double linearVelocity = 1.0;

  auto traj = aikido::planner::vectorfield::planToEndEffectorOffset(
      mStateSpace,
      mBodynode,
      mPassingConstraint,
      direction,
      distance,
      maxDistance,
      positionTolerance,
      angularTolerance,
      linearVelocity);

  EXPECT_FALSE(traj == nullptr) << "Trajectory not found";

  if (traj == nullptr)
  {
    return;
  }

  // Trajectory should have >= 1 waypoints
  EXPECT_GE(traj->getNumWaypoints(), 1)
      << "Trajectory should have >= 1 waypoints";

  // Extract and evaluate first waypoint
  auto firstWayPoint = mStateSpace->createState();
  traj->evaluate(0.0, firstWayPoint);
  Eigen::VectorXd testStart(mStateSpace->getDimension());
  Eigen::VectorXd referenceStart(mStateSpace->getDimension());
  mStateSpace->convertStateToPositions(firstWayPoint, testStart);
  mStateSpace->convertStateToPositions(startState, referenceStart);
  EXPECT_TRUE((testStart - referenceStart).norm() <= mErrorTolerance);

  int stepNum = 10;
  double timeStep = traj->getDuration() / stepNum;

  for (double t = traj->getStartTime(); t <= traj->getEndTime(); t += timeStep)
  {
    auto waypoint = mStateSpace->createState();
    traj->evaluate(t, waypoint);

    mStateSpace->setState(waypoint);
    Eigen::Isometry3d waypointTrans = mBodynode->getTransform();
    Eigen::Vector3d waypointVec = waypointTrans.translation();

    Eigen::Vector3d linear_error
        = startVec + direction * distance - waypointVec;
    Eigen::Vector3d const linear_orthogonal_error
        = linear_error - linear_error.dot(direction) * direction;
    double const linear_orthogonal_magnitude = linear_orthogonal_error.norm();
    EXPECT_TRUE(linear_orthogonal_magnitude <= positionTolerance);
  }

  auto endpoint = mStateSpace->createState();
  traj->evaluate(traj->getEndTime(), endpoint);
  mStateSpace->setState(endpoint);
  Eigen::Isometry3d endTrans = mBodynode->getTransform();
  double movedDistance
      = (endTrans.translation() - startTrans.translation()).norm();

  // Verify the moving distance
  EXPECT_TRUE(movedDistance >= distance);
  EXPECT_TRUE(movedDistance <= maxDistance);
}

TEST_F(VectorFieldPlannerTest, DirectionZeroVector)
{
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  double distance = 0.2;

  mStateSpace->getMetaSkeleton()->setPositions(mStartConfig);

  EXPECT_THROW(
      aikido::planner::vectorfield::planToEndEffectorOffset(
          mStateSpace,
          mBodynode,
          mPassingConstraint,
          direction,
          distance,
          mLinearVelocity),
      std::runtime_error);
}

TEST_F(VectorFieldPlannerTest, PlanToEndEffectorPoseTest)
{
  using dart::math::eulerXYXToMatrix;
  using aikido::planner::vectorfield::computeGeodesicDistance;

  Eigen::VectorXd goalConfig = Eigen::VectorXd::Zero(mNumDof);
  goalConfig << 1.13746, -0.363612, 0.77876, 1.07515, 1.58646, -1.00034,
      -0.03533;
  mStateSpace->getMetaSkeleton()->setPositions(goalConfig);
  Eigen::Isometry3d targetPose = mBodynode->getTransform();

  double poseErrorTolerance = 0.01;

  mStateSpace->getMetaSkeleton()->setPositions(mStartConfig);

  auto traj1 = aikido::planner::vectorfield::planToEndEffectorPose(
      mStateSpace,
      mBodynode,
      mPassingConstraint,
      targetPose,
      poseErrorTolerance);

  EXPECT_FALSE(traj1 == nullptr) << "Trajectory not found";

  if (traj1 == nullptr)
  {
    return;
  }

  // Extract and evalute first waypoint
  auto firstWayPoint = mStateSpace->createState();
  traj1->evaluate(0.0, firstWayPoint);
  Eigen::VectorXd testStart(mNumDof);
  mStateSpace->convertStateToPositions(firstWayPoint, testStart);

  double errorTolerance = 1e-3;
  EXPECT_TRUE((testStart - mStartConfig).norm() <= errorTolerance);

  auto endpoint = mStateSpace->createState();
  traj1->evaluate(traj1->getEndTime(), endpoint);
  mStateSpace->setState(endpoint);
  Eigen::Isometry3d endTrans = mBodynode->getTransform();

  double poseError = computeGeodesicDistance(endTrans, targetPose);
  EXPECT_TRUE(poseError <= poseErrorTolerance);
}
