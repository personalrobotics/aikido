#include <gtest/gtest.h>
#include <tuple>
#include <dart/dart.hpp>
#include <aikido/constraint/NonColliding.hpp>
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
  using NonColliding = aikido::constraint::NonColliding;
  using DistanceMetric = aikido::distance::DistanceMetric;
  using MetaSkeletonStateSpacePtr
      = aikido::statespace::dart::MetaSkeletonStateSpacePtr;
  using MetaSkeletonStateSpace
      = aikido::statespace::dart::MetaSkeletonStateSpace;
  using SO2 = aikido::statespace::SO2;
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
    SkeletonPtr skel
        = createNLinkRobot(3, Eigen::Vector3d(0.2, 0.2, 1.0), BALL);
    mNumDof = skel->getNumDofs();
    mLinearVelocity = 0.2;

    mUpperPositionLimits = Eigen::VectorXd::Constant(mNumDof, constantsd::pi());
    mLowerPositionLimits
        = Eigen::VectorXd::Constant(mNumDof, -constantsd::pi());
    mUpperVelocityLimits = Eigen::VectorXd::Constant(mNumDof, 2.0);
    mLowerVelocityLimits = Eigen::VectorXd::Constant(mNumDof, -2.0);

    skel->setPositionUpperLimits(mUpperPositionLimits);
    skel->setPositionLowerLimits(mLowerPositionLimits);
    skel->setVelocityUpperLimits(mUpperVelocityLimits);
    skel->setVelocityLowerLimits(mLowerVelocityLimits);
    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(skel);
    mBodynode = mStateSpace->getMetaSkeleton()->getBodyNodes().back();

    mStartConfig = Eigen::VectorXd::Zero(mNumDof);
    mStartConfig << 2.13746, -0.663612, 1.77876, 1.87515, 2.58646, -1.90034,
        -1.03533, 1.68534, -1.39628;

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

  /// Add an end-effector to the last link of the given robot
  void addEndEffector(SkeletonPtr _robot, BodyNode* _parentNode, Vector3d _dim)
  {
    // Create the end-effector node with a random dimension
    BodyNode::Properties node(BodyNode::AspectProperties("ee"));
    std::shared_ptr<Shape> shape
        = std::make_shared<BoxShape>(Vector3d(0.2, 0.2, 0.2));

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(0.0, 0.0, _dim(2)));
    Joint::Properties joint("eeJoint", T);

    auto pair = _robot->createJointAndBodyNodePair<WeldJoint>(
        _parentNode, joint, node);
    auto bodyNode = pair.second;
    bodyNode
        ->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
            shape);
  }

  //==============================================================================
  /// Creates a N link manipulator with the given dimensions where each joint is
  /// the specified type
  SkeletonPtr createNLinkRobot(
      int _n, Vector3d _dim, TypeOfDOF _type, bool _finished = false)
  {
    assert(_n > 0);

    SkeletonPtr robot = Skeleton::create();
    // robot->disableSelfCollision();

    // Create the first link, the joint with the ground and its shape
    BodyNode::Properties node(BodyNode::AspectProperties("link1"));
    //  node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
    std::shared_ptr<Shape> shape(new BoxShape(_dim));

    std::pair<Joint*, BodyNode*> pair1;
    if (BALL == _type)
    {
      BallJoint::Base::Properties properties(std::string("joint1"));
      pair1 = robot->createJointAndBodyNodePair<BallJoint>(
          nullptr, BallJoint::Properties(properties), node);
    }
    else
    {
      pair1 = add1DofJoint(
          robot,
          nullptr,
          node,
          "joint1",
          0.0,
          -constantsd::pi(),
          constantsd::pi(),
          _type);
    }

    Joint* joint = pair1.first;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // T.translation() = Eigen::Vector3d(0.0, 0.0, -0.5*dim(2));
    // joint->setTransformFromParentBodyNode(T);
    T.translation() = Eigen::Vector3d(0.0, 0.0, 0.5 * _dim(2));
    joint->setTransformFromChildBodyNode(T);
    // joint->setDampingCoefficient(0, 0.01);

    auto currentNode = pair1.second;
    currentNode
        ->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
            shape);

    BodyNode* parentNode = currentNode;

    // Create links iteratively
    for (int i = 1; i < _n; ++i)
    {
      std::ostringstream ssLink;
      std::ostringstream ssJoint;
      ssLink << "link" << i;
      ssJoint << "joint" << i;

      node = BodyNode::Properties(BodyNode::AspectProperties(ssLink.str()));
      //    node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
      shape = std::shared_ptr<Shape>(new BoxShape(_dim));

      std::pair<Joint*, BodyNode*> newPair;

      if (BALL == _type)
      {
        BallJoint::Base::Properties properties(ssJoint.str());
        newPair = robot->createJointAndBodyNodePair<BallJoint>(
            parentNode, BallJoint::Properties(properties), node);
      }
      else
      {
        newPair = add1DofJoint(
            robot,
            parentNode,
            node,
            ssJoint.str(),
            0.0,
            -constantsd::pi(),
            constantsd::pi(),
            _type);
      }

      Joint* joint = newPair.first;
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.translation() = Eigen::Vector3d(0.0, 0.0, -0.5 * _dim(2));
      joint->setTransformFromParentBodyNode(T);
      T.translation() = Eigen::Vector3d(0.0, 0.0, 0.5 * _dim(2));
      joint->setTransformFromChildBodyNode(T);
      // joint->setDampingCoefficient(0, 0.01);

      auto current_node = newPair.second;
      current_node
          ->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
              shape);

      parentNode = current_node;
    }

    // If finished, initialize the skeleton
    if (_finished)
      addEndEffector(robot, parentNode, _dim);

    return robot;
  }

  // Parameters
  double mErrorTolerance;

  // DART setup
  SkeletonPtr mSkel;
  std::vector<std::pair<JointPtr, BodyNodePtr>> mJnBn;
  MetaSkeletonStateSpacePtr mStateSpace;
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

  Eigen::VectorXd currentConfig = Eigen::VectorXd::Random(mNumDof);
  mStateSpace->getMetaSkeleton()->setPositions(currentConfig);

  Eigen::Isometry3d currentTrans = mBodynode->getTransform();
  Eigen::Vector3d currentTranslation = currentTrans.translation();

  Eigen::Vector6d desiredTwist;
  Eigen::Vector3d linearVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d angularVel = Eigen::Vector3d::Zero();
  linearVel << 0.5, 0.5, 0.5;
  desiredTwist.head<3>() = angularVel;
  desiredTwist.tail<3>() = linearVel;

  double timestep = 0.01;
  double padding = 1e-3;
  double optimizationTolerance = 1e-10;
  Eigen::VectorXd qd = Eigen::VectorXd::Zero(mNumDof);
  EXPECT_TRUE(
      computeJointVelocityFromTwist(
          desiredTwist,
          mStateSpace,
          mBodynode,
          optimizationTolerance,
          timestep,
          padding,
          qd));

  Eigen::VectorXd nextConfig(mNumDof);
  auto currentState = mStateSpace->createState();
  mStateSpace->convertPositionsToState(currentConfig, currentState);
  auto deltaState = mStateSpace->createState();
  mStateSpace->convertPositionsToState(timestep * qd, deltaState);
  auto nextState = mStateSpace->createState();
  mStateSpace->compose(currentState, deltaState, nextState);
  mStateSpace->convertStateToPositions(nextState, nextConfig);
  mStateSpace->getMetaSkeleton()->setPositions(nextConfig);

  Eigen::Isometry3d nextTrans = mBodynode->getTransform();
  Eigen::Vector3d nextTranslation = nextTrans.translation();

  Eigen::Vector3d linearDelta = nextTranslation - currentTranslation;
  Eigen::Vector3d expectedLinearDelta = linearVel * timestep;

  double tolerance = 1e-3;
  EXPECT_TRUE((linearDelta - expectedLinearDelta).norm() < tolerance);
}

TEST_F(VectorFieldPlannerTest, PlanToEndEffectorOffsetTest)
{
  Eigen::Vector3d direction;
  direction << 1., 1., 0.;
  direction.normalize();
  double distance = 0.5;

  mStateSpace->getMetaSkeleton()->setPositions(mStartConfig);
  auto startState = mStateSpace->createState();
  mStateSpace->convertPositionsToState(mStartConfig, startState);
  Eigen::Isometry3d startTrans = mBodynode->getTransform();
  Eigen::VectorXd startVec = startTrans.translation();

  double positionTolerance = 0.01;
  double angularTolerance = 0.15;
  double timestep = 0.02;
  double linearGain = 10.;
  double angularGain = 0.; // not trying to correct angular deviation

  auto traj = aikido::planner::vectorfield::planToEndEffectorOffset(
      mStateSpace,
      mBodynode,
      mPassingConstraint,
      direction,
      distance,
      mLinearVelocity,
      positionTolerance,
      angularTolerance,
      linearGain,
      angularGain,
      timestep);

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
  EXPECT_NEAR(movedDistance, distance, positionTolerance);
}

TEST_F(VectorFieldPlannerTest, DirectionZeroVector)
{
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  double distance = 0.2;

  mStateSpace->getMetaSkeleton()->setPositions(mStartConfig);

  EXPECT_THROW(
      aikido::planner::vectorfield::planToEndEffectorOffset(
          mStateSpace, mBodynode, mPassingConstraint, direction, distance, mLinearVelocity),
      std::runtime_error);
}
