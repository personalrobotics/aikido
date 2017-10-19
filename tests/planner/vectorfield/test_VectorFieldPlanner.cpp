#include <gtest/gtest.h>
#include <tuple>
#include <dart/dart.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
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
  using MetaSkeletonStateSpace
      = aikido::statespace::dart::MetaSkeletonStateSpace;
  using SO2 = aikido::statespace::SO2;
  using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
  using ScopedState = MetaSkeletonStateSpace::ScopedState;

  using BodyNode = dart::dynamics::BodyNode;
  using Joint = dart::dynamics::Joint;
  using Skeleton = dart::dynamics::Skeleton;
  using Shape = dart::dynamics::Shape;
  using BoxShape = dart::dynamics::BoxShape;
  using BodyNodePtr = dart::dynamics::BodyNodePtr;
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

  VectorFieldPlannerTest() : errorTolerance(1e-3)
  {
    SkeletonPtr skel = createNLinkRobot(
          3,
          Eigen::Vector3d(0.2, 0.2, 1.0),
          BALL);

    upperPositionLimits = Eigen::VectorXd::Constant(skel->getNumDofs(), constantsd::pi());
    lowerPositionLimits = Eigen::VectorXd::Constant(skel->getNumDofs(), -constantsd::pi());
    upperVelocityLimits = Eigen::VectorXd::Constant(skel->getNumDofs(), 2.0);
    lowerVelocityLimits = Eigen::VectorXd::Constant(skel->getNumDofs(), -2.0);

    skel->setPositionUpperLimits(upperPositionLimits);
    skel->setPositionLowerLimits(lowerPositionLimits);
    skel->setVelocityUpperLimits(upperVelocityLimits);
    skel->setVelocityLowerLimits(lowerVelocityLimits);
    stateSpace = std::make_shared<MetaSkeletonStateSpace>(skel);
    startVec = Eigen::VectorXd(skel->getNumDofs());
    //startVec = Eigen::VectorXd::Random(skel->getNumDofs());
    //startVec *= constantsd::pi();
    startVec << 2.13746, -0.663612, 1.77876, 1.87515, 2.58646,
                -1.90034, -1.03533, 1.68534, -1.39628;

    goalVec = Eigen::VectorXd(skel->getNumDofs());
    passingConstraint = std::make_shared<PassingConstraint>(stateSpace);
    failingConstraint = std::make_shared<FailingConstraint>(stateSpace);
    interpolator = std::make_shared<GeodesicInterpolator>(stateSpace);
  }

  std::pair<Joint*, BodyNode*> add1DofJoint(
      SkeletonPtr skel,
      BodyNode* parent,
      const BodyNode::Properties& node,
      const std::string& name,
      double val,
      double min,
      double max,
      int type)
  {
    dart::dynamics::GenericJoint<dart::math::R1Space>::Properties properties(
        name);
    properties.mPositionLowerLimits[0] = min;
    properties.mPositionUpperLimits[0] = max;
    std::pair<Joint*, BodyNode*> newComponent;
    if (DOF_X == type)
      newComponent = skel->createJointAndBodyNodePair<PrismaticJoint>(
          parent,
          PrismaticJoint::Properties(
              properties, Eigen::Vector3d(1.0, 0.0, 0.0)),
          node);
    else if (DOF_Y == type)
      newComponent = skel->createJointAndBodyNodePair<PrismaticJoint>(
          parent,
          PrismaticJoint::Properties(
              properties, Eigen::Vector3d(0.0, 1.0, 0.0)),
          node);
    else if (DOF_Z == type)
      newComponent = skel->createJointAndBodyNodePair<PrismaticJoint>(
          parent,
          PrismaticJoint::Properties(
              properties, Eigen::Vector3d(0.0, 0.0, 1.0)),
          node);
    else if (DOF_YAW == type)
      newComponent = skel->createJointAndBodyNodePair<RevoluteJoint>(
          parent,
          RevoluteJoint::Properties(properties, Eigen::Vector3d(0.0, 0.0, 1.0)),
          node);
    else if (DOF_PITCH == type)
      newComponent = skel->createJointAndBodyNodePair<RevoluteJoint>(
          parent,
          RevoluteJoint::Properties(properties, Eigen::Vector3d(0.0, 1.0, 0.0)),
          node);
    else if (DOF_ROLL == type)
      newComponent = skel->createJointAndBodyNodePair<RevoluteJoint>(
          parent,
          RevoluteJoint::Properties(properties, Eigen::Vector3d(1.0, 0.0, 0.0)),
          node);

    newComponent.first->setPosition(0, val);
    return newComponent;
  }

  /// Add an end-effector to the last link of the given robot
  void addEndEffector(SkeletonPtr robot, BodyNode* parent_node, Vector3d dim)
  {
    // Create the end-effector node with a random dimension
    BodyNode::Properties node(BodyNode::AspectProperties("ee"));
    std::shared_ptr<Shape> shape
        = std::make_shared<BoxShape>(Vector3d(0.2, 0.2, 0.2));

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(0.0, 0.0, dim(2)));
    Joint::Properties joint("eeJoint", T);

    auto pair = robot->createJointAndBodyNodePair<WeldJoint>(
        parent_node, joint, node);
    auto bodyNode = pair.second;
    bodyNode
        ->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
            shape);
  }

  //==============================================================================
  /// Creates a N link manipulator with the given dimensions where each joint is
  /// the specified type
  SkeletonPtr createNLinkRobot(int _n,
                               Vector3d dim,
                               TypeOfDOF type,
                               bool finished = false)
  {
    assert(_n > 0);

    SkeletonPtr robot = Skeleton::create();
    //robot->disableSelfCollision();

    // Create the first link, the joint with the ground and its shape
    BodyNode::Properties node(BodyNode::AspectProperties("link1"));
  //  node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
    std::shared_ptr<Shape> shape(new BoxShape(dim));

    std::pair<Joint*, BodyNode*> pair1;
    if (BALL == type)
    {
      BallJoint::Base::Properties properties(std::string("joint1"));
      pair1 = robot->createJointAndBodyNodePair<BallJoint>(
            nullptr, BallJoint::Properties(properties), node);
    }
    else
    {
      pair1 = add1DofJoint(robot, nullptr, node, "joint1", 0.0,
                           -constantsd::pi(), constantsd::pi(), type);
    }

    Joint* joint = pair1.first;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    // T.translation() = Eigen::Vector3d(0.0, 0.0, -0.5*dim(2));
    // joint->setTransformFromParentBodyNode(T);
    T.translation() = Eigen::Vector3d(0.0, 0.0, 0.5*dim(2));
    joint->setTransformFromChildBodyNode(T);
    //joint->setDampingCoefficient(0, 0.01);

    auto current_node = pair1.second;
    current_node->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          shape);

    BodyNode* parent_node = current_node;

    // Create links iteratively
    for (int i = 1; i < _n; ++i)
    {
      std::ostringstream ssLink;
      std::ostringstream ssJoint;
      ssLink << "link" << i;
      ssJoint << "joint" << i;

      node = BodyNode::Properties(BodyNode::AspectProperties(ssLink.str()));
  //    node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim(2)/2.0));
      shape = std::shared_ptr<Shape>(new BoxShape(dim));

      std::pair<Joint*, BodyNode*> newPair;

      if (BALL == type)
      {
        BallJoint::Base::Properties properties(ssJoint.str());
        newPair = robot->createJointAndBodyNodePair<BallJoint>(
              parent_node,
              BallJoint::Properties(properties),
              node);
      }
      else
      {
        newPair = add1DofJoint(robot, parent_node, node, ssJoint.str(), 0.0,
                               -constantsd::pi(), constantsd::pi(), type);
      }

      Joint* joint = newPair.first;
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.translation() = Eigen::Vector3d(0.0, 0.0, -0.5*dim(2));
      joint->setTransformFromParentBodyNode(T);
      T.translation() = Eigen::Vector3d(0.0, 0.0, 0.5*dim(2));
      joint->setTransformFromChildBodyNode(T);
      //joint->setDampingCoefficient(0, 0.01);

      auto current_node = newPair.second;
      current_node->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
            shape);

      parent_node = current_node;
    }

    // If finished, initialize the skeleton
    if(finished)
      addEndEffector(robot, parent_node, dim);

    return robot;
  }


  // Parameters
  double errorTolerance;

  // DART setup
  SkeletonPtr skel;
  std::vector<std::pair<JointPtr, BodyNodePtr>> jn_bn;
  shared_ptr<MetaSkeletonStateSpace> stateSpace;
  Eigen::VectorXd upperPositionLimits;
  Eigen::VectorXd lowerPositionLimits;
  Eigen::VectorXd upperVelocityLimits;
  Eigen::VectorXd lowerVelocityLimits;

  // Arguments for planner
  Eigen::VectorXd startVec;
  Eigen::VectorXd goalVec;
  shared_ptr<PassingConstraint> passingConstraint;
  shared_ptr<FailingConstraint> failingConstraint;
  shared_ptr<GeodesicInterpolator> interpolator;
  aikido::planner::PlanningResult planningResult;
};

TEST_F(VectorFieldPlannerTest, PlanToEndEffectorOffsetTest)
{
  Eigen::Vector3d direction;
  direction << 0., 1., 0.;
  direction.normalize();
  double distance = 0.1;

  stateSpace->getMetaSkeleton()->setPositions(startVec);
  auto startState = stateSpace->createState();
  stateSpace->convertPositionsToState(startVec, startState);
  dart::dynamics::BodyNodePtr bn
      = stateSpace->getMetaSkeleton()->getBodyNodes().back();
  Eigen::Isometry3d startTrans = bn->getTransform();
  Eigen::VectorXd startVec = startTrans.translation();

  double position_tolerance = 0.01;
  double angular_tolerance = 0.01;
  double duration = 1.0;
  double timestep = 0.2;
  double linear_gain = 10.;
  double angular_gain = 10.;

  auto traj = aikido::planner::vectorfield::planToEndEffectorOffset(
      stateSpace,
      bn,
      passingConstraint,
      direction,
      distance,
      position_tolerance,
      angular_tolerance,
      duration,
      timestep,
      linear_gain,
      angular_gain);

  EXPECT_FALSE(traj == nullptr) << "Trajectory not found";

  if (traj == nullptr)
  {
    return;
  }

  // Trajectory should have >= 1 waypoints
  EXPECT_GE(traj->getNumWaypoints(), 1)
      << "Trajectory should have >= 1 waypoints";

  // Extract and evaluate first waypoint
  auto firstWayPoint = stateSpace->createState();
  traj->evaluate(0.0, firstWayPoint);
  Eigen::VectorXd testStart(stateSpace->getDimension());
  Eigen::VectorXd referenceStart(stateSpace->getDimension());
  stateSpace->convertStateToPositions(firstWayPoint, testStart);
  stateSpace->convertStateToPositions(startState, referenceStart);
  EXPECT_TRUE((testStart - referenceStart).norm() <= errorTolerance);

  double expectedDistance = 0.0;
  int stepNum = 10;
  double timeStep = traj->getDuration() / stepNum;

  for (double t = traj->getStartTime(); t <= traj->getEndTime(); t += timeStep)
  {
    auto waypoint = stateSpace->createState();
    traj->evaluate(t, waypoint);

    stateSpace->setState(waypoint);
    auto waypointTrans
        = stateSpace->getMetaSkeleton()->getBodyNodes().back()->getTransform();
    auto waypointVec = waypointTrans.translation();

    // update distance
    expectedDistance = (startVec - waypointVec).norm();

    Eigen::Isometry3d expectedTrans = startTrans;
    expectedTrans.translation() += expectedDistance * direction.normalized();
    auto expectedVec = expectedTrans.translation();

    double currentError = (waypointVec - expectedVec).norm();
    // Verify that the position is monotone
    EXPECT_TRUE(currentError <= errorTolerance);
  }

  // Verify the moving distance
  EXPECT_NEAR(expectedDistance, distance, errorTolerance);
}

TEST_F(VectorFieldPlannerTest, DirectionZeroVector)
{
  Eigen::Vector3d direction;
  direction << 0., 0., 0.;
  double distance = 0.2;

  stateSpace->getMetaSkeleton()->setPositions(startVec);
  auto startState = stateSpace->createState();
  stateSpace->convertPositionsToState(startVec, startState);
  dart::dynamics::BodyNodePtr bn
      = stateSpace->getMetaSkeleton()->getBodyNodes().back();

  EXPECT_THROW(
      aikido::planner::vectorfield::planToEndEffectorOffset(
          stateSpace, bn, passingConstraint, direction, distance),
      std::runtime_error);
}
