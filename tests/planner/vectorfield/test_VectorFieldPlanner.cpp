#include <tuple>

#include <dart/dart.hpp>
#include <gtest/gtest.h>

#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/dart/ConfigurationToEndEffectorOffset.hpp>
#include <aikido/planner/vectorfield/MoveEndEffectorOffsetVectorField.hpp>
#include <aikido/planner/vectorfield/VectorFieldConfigurationToEndEffectorOffsetPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

#include "../../constraint/MockConstraints.hpp"

using std::make_shared;
using std::shared_ptr;

class VectorFieldPlannerTest : public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
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
    mSkel = createNLinkRobot();
    mNumDof = mSkel->getNumDofs();

    mUpperPositionLimits = Eigen::VectorXd::Constant(mNumDof, constantsd::pi());
    mLowerPositionLimits
        = Eigen::VectorXd::Constant(mNumDof, -constantsd::pi());
    mUpperVelocityLimits = Eigen::VectorXd::Constant(mNumDof, 4.0);
    mLowerVelocityLimits = Eigen::VectorXd::Constant(mNumDof, -4.0);

    mSkel->setPositionUpperLimits(mUpperPositionLimits);
    mSkel->setPositionLowerLimits(mLowerPositionLimits);
    mSkel->setVelocityUpperLimits(mUpperVelocityLimits);
    mSkel->setVelocityLowerLimits(mLowerVelocityLimits);
    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(mSkel.get());
    mBodynode = mSkel->getBodyNodes().back();

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

  Eigen::VectorXd jointVelocityUpperLimits = mSkel->getVelocityUpperLimits();
  Eigen::VectorXd jointVelocityLowerLimits = mSkel->getVelocityLowerLimits();

  double maxStepSize = 0.01;
  double padding = 2e-3;

  // Linear only
  Eigen::Vector6d desiredTwist1 = Eigen::Vector6d::Zero();
  desiredTwist1.tail<3>() << 1.0, 1.0, 1.0;

  Eigen::VectorXd qd = Eigen::VectorXd::Zero(mNumDof);
  EXPECT_TRUE(computeJointVelocityFromTwist(
      qd,
      desiredTwist1,
      mSkel,
      mBodynode,
      padding,
      jointVelocityLowerLimits,
      jointVelocityUpperLimits,
      true,
      maxStepSize));

  // Angular only
  Eigen::Vector6d desiredTwist2 = Eigen::Vector6d::Zero();
  desiredTwist2.head<3>() << 1.0, 1.0, 1.0;

  EXPECT_TRUE(computeJointVelocityFromTwist(
      qd,
      desiredTwist2,
      mSkel,
      mBodynode,
      padding,
      jointVelocityLowerLimits,
      jointVelocityUpperLimits,
      true,
      maxStepSize));

  // Both linear and angular
  Eigen::Vector6d desiredTwist3 = Eigen::Vector6d::Zero();
  desiredTwist3.head<3>() << 1.0, 1.0, 1.0;
  desiredTwist3.tail<3>() << 1.0, 1.0, 1.0;

  EXPECT_TRUE(computeJointVelocityFromTwist(
      qd,
      desiredTwist2,
      mSkel,
      mBodynode,
      padding,
      jointVelocityLowerLimits,
      jointVelocityUpperLimits,
      true,
      maxStepSize));
}

TEST_F(VectorFieldPlannerTest, PlanToEndEffectorOffsetTest)
{
  using aikido::planner::dart::ConfigurationToEndEffectorOffset;
  using aikido::planner::vectorfield::
      VectorFieldConfigurationToEndEffectorOffsetPlanner;

  Eigen::Vector3d direction;
  direction << 1., 1., 0.;
  direction.normalize();
  double signedDistance = 0.41;
  double distanceTolerance = 0.01;

  mSkel->setPositions(mStartConfig);
  Eigen::Isometry3d startTrans = mBodynode->getTransform();
  Eigen::VectorXd startVec = startTrans.translation();

  double positionTolerance = 0.01;
  double angularTolerance = 0.15;
  double initialStepSize = 0.01;
  double jointLimitTolerance = 1e-3;
  double constraintCheckResolution = 1e-2;
  std::chrono::duration<double> timelimit(60.0);

  // Create problem.
  auto offsetProblem = ConfigurationToEndEffectorOffset(
      mStateSpace,
      mSkel,
      mBodynode,
      direction,
      signedDistance,
      mPassingConstraint);

  // Create planner.
  VectorFieldConfigurationToEndEffectorOffsetPlanner vfOffsetPlanner(
      mStateSpace,
      mSkel,
      distanceTolerance,
      positionTolerance,
      angularTolerance,
      initialStepSize,
      jointLimitTolerance,
      constraintCheckResolution,
      timelimit);

  // Invoke planning method.
  auto traj = vfOffsetPlanner.plan(offsetProblem);

  EXPECT_FALSE(traj == nullptr) << "Trajectory not found";

  if (traj == nullptr)
  {
    return;
  }

  // Extract and evaluate first waypoint
  auto firstWayPoint = mStateSpace->createState();
  traj->evaluate(0.0, firstWayPoint);
  Eigen::VectorXd testStart(mStateSpace->getDimension());
  Eigen::VectorXd referenceStart(mStateSpace->getDimension());
  mStateSpace->convertStateToPositions(firstWayPoint, testStart);
  EXPECT_LE((testStart - mStartConfig).norm(), mErrorTolerance);

  int stepNum = 10;
  double timeStep = traj->getDuration() / stepNum;

  double minDistance = signedDistance - distanceTolerance;
  double maxDistance = signedDistance + distanceTolerance;

  for (double t = traj->getStartTime(); t <= traj->getEndTime(); t += timeStep)
  {
    auto waypoint = mStateSpace->createState();
    traj->evaluate(t, waypoint);

    mStateSpace->setState(mSkel.get(), waypoint);
    Eigen::Isometry3d waypointTrans = mBodynode->getTransform();
    Eigen::Vector3d waypointVec = waypointTrans.translation();

    Eigen::Vector3d linear_error
        = startVec + direction * minDistance - waypointVec;
    Eigen::Vector3d const linear_orthogonal_error
        = linear_error - linear_error.dot(direction) * direction;
    double const linear_orthogonal_magnitude = linear_orthogonal_error.norm();
    EXPECT_LE(linear_orthogonal_magnitude, positionTolerance);
  }

  auto endpoint = mStateSpace->createState();
  traj->evaluate(traj->getEndTime(), endpoint);
  mStateSpace->setState(mSkel.get(), endpoint);
  Eigen::Isometry3d endTrans = mBodynode->getTransform();
  double movedDistance
      = (endTrans.translation() - startTrans.translation()).norm();

  // Verify the moving distance
  EXPECT_GE(movedDistance, minDistance);
  EXPECT_LE(movedDistance, maxDistance);
}

TEST_F(VectorFieldPlannerTest, DirectionZeroVector)
{
  using aikido::planner::dart::ConfigurationToEndEffectorOffset;
  using aikido::planner::vectorfield::
      VectorFieldConfigurationToEndEffectorOffsetPlanner;

  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  double signedDistance = 0.21;

  mSkel->setPositions(mStartConfig);

  // Create Problem, which should fail.
  EXPECT_THROW(
      ConfigurationToEndEffectorOffset(
          mStateSpace,
          mSkel,
          mBodynode,
          direction,
          signedDistance,
          mPassingConstraint),
      std::invalid_argument);
}

TEST_F(VectorFieldPlannerTest, PlanToEndEffectorPoseTest)
{
  using aikido::planner::vectorfield::computeGeodesicDistance;
  using aikido::planner::vectorfield::MoveEndEffectorOffsetVectorField;
  using dart::math::eulerXYXToMatrix;

  Eigen::VectorXd goalConfig = Eigen::VectorXd::Zero(mNumDof);
  goalConfig << 1.13746, -0.363612, 0.77876, 1.07515, 1.58646, -1.00034,
      -0.03533;
  mSkel->setPositions(goalConfig);
  Eigen::Isometry3d targetPose = mBodynode->getTransform();

  double poseErrorTolerance = 0.01;
  double initialStepSize = 0.05;
  double r = 1.0;
  double jointLimitTolerance = 1e-2;
  double constraintCheckResolution = 1e-2;
  std::chrono::duration<double> timelimit(300.);

  mSkel->setPositions(mStartConfig);

  auto traj1 = aikido::planner::vectorfield::planToEndEffectorPose(
      mStateSpace,
      mSkel,
      mBodynode,
      mPassingConstraint,
      targetPose,
      poseErrorTolerance,
      r,
      initialStepSize,
      jointLimitTolerance,
      constraintCheckResolution,
      timelimit);

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
  mStateSpace->setState(mSkel.get(), endpoint);
  Eigen::Isometry3d endTrans = mBodynode->getTransform();

  double poseError = computeGeodesicDistance(endTrans, targetPose, r);
  EXPECT_LE(poseError, poseErrorTolerance);
}
