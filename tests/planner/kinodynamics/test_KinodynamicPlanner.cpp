#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/kinodynamics/KinodynamicPlanner.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include "../../constraint/MockConstraints.hpp"

using std::shared_ptr;
using std::make_shared;
using aikido::planner::kinodynamics::planMinimumTimeViaConstraint;

class KinodynamicPlannerTest : public ::testing::Test
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

  KinodynamicPlannerTest()
  {
    mSkel = createNLinkRobot();
    mNumDof = mSkel->getNumDofs();

    mUpperPositionLimits = Eigen::VectorXd::Constant(mNumDof, constantsd::pi());
    mLowerPositionLimits
        = Eigen::VectorXd::Constant(mNumDof, -constantsd::pi());
    mUpperVelocityLimits = Eigen::VectorXd::Constant(mNumDof, 4.0);
    mLowerVelocityLimits = Eigen::VectorXd::Constant(mNumDof, -4.0);

    mUpperAccelerationLimits = Eigen::VectorXd::Constant(mNumDof, 1.0);
    mLowerAccelerationLimits = Eigen::VectorXd::Constant(mNumDof, -1.0);

    mSkel->setPositionUpperLimits(mUpperPositionLimits);
    mSkel->setPositionLowerLimits(mLowerPositionLimits);
    mSkel->setVelocityUpperLimits(mUpperVelocityLimits);
    mSkel->setVelocityLowerLimits(mLowerVelocityLimits);
    mSkel->setAccelerationUpperLimits(mUpperAccelerationLimits);
    mSkel->setAccelerationLowerLimits(mLowerAccelerationLimits);
    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(mSkel.get());
    mBodynode = mSkel->getBodyNodes().back();

    mPassingConstraint = std::make_shared<PassingConstraint>(mStateSpace);
    mFailingConstraint = std::make_shared<FailingConstraint>(mStateSpace);
    mMaxPlanTime = 30.;
    mMaxDistanceBtwValidityChecks = 0.01;
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
    for (int i = 1; i < 6; ++i)
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
  Eigen::VectorXd mUpperAccelerationLimits;
  Eigen::VectorXd mLowerAccelerationLimits;

  shared_ptr<PassingConstraint> mPassingConstraint;
  shared_ptr<FailingConstraint> mFailingConstraint;

  double mMaxPlanTime;
  double mMaxDistanceBtwValidityChecks;
};

TEST_F(KinodynamicPlannerTest, ReturnsStartToGoalTrajOnSuccess)
{
  Eigen::VectorXd startConfig(mNumDof);
  Eigen::VectorXd viaConfig(mNumDof);
  Eigen::VectorXd goalConfig(mNumDof);
  startConfig << 2., 2., 2., 2., 2., 2.;
  viaConfig << 0., 0., 0., 0., 0., 0.;
  goalConfig << -2., -2., -2., -2., -2., -2.;

  Eigen::Vector6d viaVelocity;
  viaVelocity << 1., 1., 1., 1., 1., 1.;

  auto startState = mStateSpace->createState();
  auto viaState = mStateSpace->createState();
  auto goalState = mStateSpace->createState();
  mStateSpace->convertPositionsToState(startConfig, startState);
  mStateSpace->convertPositionsToState(viaConfig, viaState);
  mStateSpace->convertPositionsToState(goalConfig, goalState);

  auto traj = planMinimumTimeViaConstraint(
      startState,
      goalState,
      viaState,
      viaVelocity,
      mSkel,
      mStateSpace,
      mPassingConstraint,
      mMaxPlanTime,
      mMaxDistanceBtwValidityChecks);

  EXPECT_FALSE(traj == nullptr) << "Trajectory not found";
}

TEST_F(KinodynamicPlannerTest, FailIfConstraintNotSatisfied)
{
  Eigen::VectorXd startConfig(mNumDof);
  Eigen::VectorXd viaConfig(mNumDof);
  Eigen::VectorXd goalConfig(mNumDof);
  startConfig << 2., 2., 2., 2., 2., 2.;
  viaConfig << 0., 0., 0., 0., 0., 0.;
  goalConfig << -2., -2., -2., -2., -2., -2.;

  Eigen::Vector6d viaVelocity(mNumDof);
  viaVelocity << 1., 1., 1., 1., 1., 1.;

  auto startState = mStateSpace->createState();
  auto viaState = mStateSpace->createState();
  auto goalState = mStateSpace->createState();
  mStateSpace->convertPositionsToState(startConfig, startState);
  mStateSpace->convertPositionsToState(viaConfig, viaState);
  mStateSpace->convertPositionsToState(goalConfig, goalState);

  auto traj = planMinimumTimeViaConstraint(
      startState,
      goalState,
      viaState,
      viaVelocity,
      mSkel,
      mStateSpace,
      mPassingConstraint,
      mMaxPlanTime,
      mMaxDistanceBtwValidityChecks);
  EXPECT_EQ(nullptr, traj);
}
