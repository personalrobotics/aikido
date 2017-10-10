#include <gtest/gtest.h>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include "../../constraint/MockConstraints.hpp"
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/constraint/Testable.hpp>
#include <dart/dart.hpp>
#include <tuple>

using std::shared_ptr;
using std::make_shared;

class VectorFieldPlannerTest : public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
  using NonColliding = aikido::constraint::NonColliding;
  using DistanceMetric = aikido::distance::DistanceMetric;
  using MetaSkeletonStateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
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
  using VisualAspect = dart::dynamics::VisualAspect;
  using CollisionAspect = dart::dynamics::CollisionAspect;
  using DynamicsAspect = dart::dynamics::DynamicsAspect;
  using constantsd = dart::math::constantsd;
  using Vector3d = Eigen::Vector3d;

  enum TypeOfDOF
  {
    DOF_X, DOF_Y, DOF_Z, DOF_ROLL, DOF_PITCH, DOF_YAW
  };

  VectorFieldPlannerTest()
      : errorTolerance(1e-4)
  {
    skel = createThreeLinkRobot(Eigen::Vector3d(1.0, 1.0, 1.0),
                                DOF_X,
                                Eigen::Vector3d(1.0, 1.0, 1.0),
                                DOF_Y,
                                Eigen::Vector3d(1.0, 1.0, 1.0),
                                DOF_Z,
                                false, false);
    upperPositionLimits = Eigen::VectorXd(skel->getNumDofs());
    upperPositionLimits << 3.1, 3.1, 3.1;
    lowerPositionLimits = Eigen::VectorXd(skel->getNumDofs());
    lowerPositionLimits << -3.1, -3.1, -3.1;
    upperVelocityLimits = Eigen::VectorXd(skel->getNumDofs());
    upperVelocityLimits << 10.0, 10.0, 10.0;
    lowerVelocityLimits = Eigen::VectorXd(skel->getNumDofs());
    lowerVelocityLimits << -10.0, -10.0, -10.0;

    skel->setPositionUpperLimits(upperPositionLimits);
    skel->setPositionLowerLimits(lowerPositionLimits);
    skel->setVelocityUpperLimits(upperVelocityLimits);
    skel->setVelocityLowerLimits(lowerVelocityLimits);
    stateSpace = std::make_shared<MetaSkeletonStateSpace>(skel);
    startVec = Eigen::VectorXd(skel->getNumDofs());
    goalVec = Eigen::VectorXd(skel->getNumDofs());
    passingConstraint = std::make_shared<PassingConstraint>(stateSpace);
    failingConstraint = std::make_shared<FailingConstraint>(stateSpace);
    interpolator = std::make_shared<GeodesicInterpolator>(stateSpace);


  }

  std::pair<Joint*, BodyNode*> add1DofJoint(SkeletonPtr skel,
      BodyNode* parent, const BodyNode::Properties& node,
      const std::string& name, double val, double min, double max, int type)
  {
    dart::dynamics::GenericJoint<dart::math::R1Space>::Properties properties(name);
    properties.mPositionLowerLimits[0] = min;
    properties.mPositionUpperLimits[0] = max;
    std::pair<Joint*, BodyNode*> newComponent;
    if(DOF_X == type)
      newComponent = skel->createJointAndBodyNodePair<PrismaticJoint>(parent,
        PrismaticJoint::Properties(properties, Eigen::Vector3d(1.0, 0.0, 0.0)), node);
    else if(DOF_Y == type)
      newComponent = skel->createJointAndBodyNodePair<PrismaticJoint>(parent,
        PrismaticJoint::Properties(properties, Eigen::Vector3d(0.0, 1.0, 0.0)), node);
    else if(DOF_Z == type)
      newComponent = skel->createJointAndBodyNodePair<PrismaticJoint>(parent,
        PrismaticJoint::Properties(properties, Eigen::Vector3d(0.0, 0.0, 1.0)), node);
    else if(DOF_YAW == type)
      newComponent = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,
        RevoluteJoint::Properties(properties, Eigen::Vector3d(0.0, 0.0, 1.0)), node);
    else if(DOF_PITCH == type)
      newComponent = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,
        RevoluteJoint::Properties(properties, Eigen::Vector3d(0.0, 1.0, 0.0)), node);
    else if(DOF_ROLL == type)
      newComponent = skel->createJointAndBodyNodePair<RevoluteJoint>(parent,
        RevoluteJoint::Properties(properties, Eigen::Vector3d(1.0, 0.0, 0.0)), node);

    newComponent.first->setPosition(0, val);
    return newComponent;
  }

  /// Add an end-effector to the last link of the given robot
  void addEndEffector(SkeletonPtr robot, BodyNode* parent_node, Vector3d dim)
  {
    // Create the end-effector node with a random dimension
    BodyNode::Properties node(BodyNode::AspectProperties("ee"));
    std::shared_ptr<Shape> shape = std::make_shared<BoxShape>(Vector3d(0.2, 0.2, 0.2));

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translate(Eigen::Vector3d(0.0, 0.0, dim(2)));
    Joint::Properties joint("eeJoint", T);

    auto pair = robot->createJointAndBodyNodePair<WeldJoint>(
          parent_node, joint, node);
    auto bodyNode = pair.second;
    bodyNode->createShapeNodeWith<
        VisualAspect, CollisionAspect, DynamicsAspect>(shape);
  }

  SkeletonPtr createThreeLinkRobot(Eigen::Vector3d dim1, TypeOfDOF type1,
                                   Eigen::Vector3d dim2, TypeOfDOF type2,
                                   Eigen::Vector3d dim3, TypeOfDOF type3,
                                   bool finished = false,
                                   bool collisionShape = true,
                                   size_t stopAfter = 3)
  {
    SkeletonPtr robot = Skeleton::create();

    Eigen::Vector3d dimEE = dim1;

    // Create the first link
    BodyNode::Properties node(BodyNode::AspectProperties("link1"));
    node.mInertia.setLocalCOM(Eigen::Vector3d(0.0, 0.0, dim1(2)/2.0));
    std::shared_ptr<Shape> shape = make_shared<BoxShape>(dim1);

    std::pair<Joint*, BodyNode*> pair1 = add1DofJoint(
        robot, nullptr, node, "joint1", 0.0, -constantsd::pi(), constantsd::pi(), type1);
    auto current_node = pair1.second;
    auto shapeNode = current_node->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
    if(collisionShape)
    {
      shapeNode->createCollisionAspect();
      shapeNode->createDynamicsAspect();
    }

    BodyNode* parent_node = current_node;

    if(stopAfter > 1)
    {
      // Create the second link
      node = BodyNode::Properties(BodyNode::AspectProperties("link2"));
      node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim2(2)/2.0));
      shape = std::make_shared<BoxShape>(dim2);

      std::pair<Joint*, BodyNode*> pair2 = add1DofJoint(
          robot, parent_node, node, "joint2", 0.0, -constantsd::pi(), constantsd::pi(), type2);
      Joint* joint = pair2.first;
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.translate(Eigen::Vector3d(0.0, 0.0, dim1(2)));
      joint->setTransformFromParentBodyNode(T);

      auto current_node = pair2.second;
      auto shapeNode = current_node->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
      if(collisionShape)
      {
        shapeNode->createCollisionAspect();
        shapeNode->createDynamicsAspect();
      }

      parent_node = pair2.second;
      dimEE = dim2;
    }

    if(stopAfter > 2)
    {
      // Create the third link
      node = BodyNode::Properties(BodyNode::AspectProperties("link3"));
      node.mInertia.setLocalCOM(Vector3d(0.0, 0.0, dim3(2)/2.0));
      shape = std::make_shared<BoxShape>(dim3);
      std::pair<Joint*, BodyNode*> pair3 = add1DofJoint(
            robot, parent_node, node, "joint3", 0.0, -constantsd::pi(), constantsd::pi(), type3);

      Joint* joint = pair3.first;
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.translate(Eigen::Vector3d(0.0, 0.0, dim2(2)));
      joint->setTransformFromParentBodyNode(T);

      auto current_node = pair3.second;
      auto shapeNode = current_node->createShapeNodeWith<VisualAspect>(shape);
      if(collisionShape)
      {
        shapeNode->createCollisionAspect();
        shapeNode->createDynamicsAspect();
      }

      parent_node = pair3.second;
      dimEE = dim3;
    }

    // If finished, add an end effector
    if(finished)
      addEndEffector(robot, parent_node, dimEE);

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
  direction << 0., 0., 1.;
  double distance = 1.0;

  stateSpace->getMetaSkeleton()->setPositions(startVec);
  auto startState = stateSpace->createState();
  stateSpace->convertPositionsToState(startVec, startState);
  dart::dynamics::BodyNodePtr bn = stateSpace->getMetaSkeleton()->getBodyNodes().back();
  Eigen::Isometry3d startTrans = bn->getTransform();
  Eigen::VectorXd startVec = startTrans.translation();

  auto traj = aikido::planner::vectorfield::planToEndEffectorOffset(stateSpace,
                                                                    bn,
                                                                    passingConstraint,
                                                                    direction,
                                                                    distance);

  EXPECT_FALSE(traj==nullptr) << "Trajectory not found";

  //Trajectory should have >= 1 waypoints
  EXPECT_GE(traj->getNumWaypoints(),1)
          << "Trajectory should have >= 1 waypoints";

  // Extract and evaluate first waypoint
  auto firstWayPoint = stateSpace->createState();
  traj->evaluate(0.0, firstWayPoint);
  Eigen::VectorXd testStart(stateSpace->getDimension());
  Eigen::VectorXd referenceStart(stateSpace->getDimension());
  stateSpace->convertStateToPositions(firstWayPoint, testStart);
  stateSpace->convertStateToPositions(startState, referenceStart);
  EXPECT_TRUE( (testStart-referenceStart).norm() <= errorTolerance);

    double expectedDistance = 0.0;
    int stepNum = 10;
    double timeStep = traj->getDuration() / stepNum;
    for(double t=traj->getStartTime(); t<=traj->getEndTime(); t+=timeStep)
    {
      auto waypoint = stateSpace->createState();
      traj->evaluate(t, waypoint);

      stateSpace->setState(waypoint);
      auto waypointTrans = stateSpace->getMetaSkeleton()->getBodyNodes().back()->getTransform();
      auto waypointVec = waypointTrans.translation();

      // update distance
      expectedDistance = (startVec - waypointVec).norm();

      Eigen::Isometry3d expectedTrans = startTrans;
      expectedTrans.translation() += expectedDistance * direction;
      auto expectedVec = expectedTrans.translation();

      // Verify that the position is monotone
      EXPECT_TRUE( (waypointVec-expectedVec).norm() <= errorTolerance );
    }

    // Verify the moving distance
    EXPECT_NEAR(expectedDistance, distance, errorTolerance);

}

TEST_F(VectorFieldPlannerTest, PlanToEndEffectorOffsetCollisionTest)
{


}
