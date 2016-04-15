#include <random>
#include <aikido/constraint/FkTestable.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/constraint/CollisionConstraint.hpp>
#include <aikido/constraint/dart.hpp>
#include <aikido/constraint/IkSampleableConstraint.hpp>
#include <aikido/constraint/FkTestable.hpp>
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>
#include <aikido/distance/DistanceMetric.hpp>
#include <aikido/distance/DistanceMetricDefaults.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/ompl/OMPLPlanner.hpp>
#include <aikido/util/RNG.hpp>
#include <dart/dart.h>
#include <gtest/gtest.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <Eigen/Dense>

using dart::common::make_unique;
using aikido::util::RNGWrapper;
using aikido::util::RNG;
using namespace aikido::constraint;
using namespace aikido::distance;
using namespace aikido::statespace;
using namespace dart::dynamics;

using DefaultRNG = RNGWrapper<std::default_random_engine>;

static std::unique_ptr<DefaultRNG> make_rng()
{
  return make_unique<RNGWrapper<std::default_random_engine>>(0);
}

class OMPLGoalSetTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto robot = Skeleton::create("robot");

    // Root joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitZ();
    properties1.mName = "j1";
    auto bn1 = robot->createJointAndBodyNodePair<RevoluteJoint>(
                          nullptr, properties1,
                          BodyNode::Properties(std::string("b1"))).second;

    // Joint 2
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitZ();
    properties2.mName = "j2";
    properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 1, 0);
    auto bn2 = robot->createJointAndBodyNodePair<RevoluteJoint>(
                          bn1, properties2,
                          BodyNode::Properties(std::string("b2"))).second;

    // End effector
    RevoluteJoint::Properties propertiesEE;
    propertiesEE.mAxis = Eigen::Vector3d::UnitZ();
    propertiesEE.mName = "ee";
    propertiesEE.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 1, 0);
    endEffector = robot->createJointAndBodyNodePair<RevoluteJoint>(
                             bn2, propertiesEE,
                             BodyNode::Properties(std::string("b3"))).second;

    // Statespace
    stateSpace = std::make_shared<MetaSkeletonStateSpace>(robot);
    interpolator = std::make_shared<GeodesicInterpolator>(stateSpace);

    // Goal region
    auto T0_w = Eigen::Isometry3d::Identity();
    auto Tw_e = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 2> Bw;
    Bw << 1, 2, -0.5, 0.5, 0., 0., 0, 0, 0, 0, -M_PI, M_PI;
    goalTSR = std::make_shared<TSR>(T0_w, Bw, Tw_e);

    auto seedSampler =
        aikido::constraint::createSampleableBounds(stateSpace, make_rng());
    auto ik = dart::dynamics::InverseKinematics::create(endEffector);
    goalSampleable = std::make_shared<IkSampleableConstraint>(
        stateSpace, goalTSR, std::move(seedSampler), ik, make_rng(), 30);
    goalTestable =
        std::make_shared<FkTestable>(stateSpace, endEffector.get(), goalTSR);

    // Collision constraint
    auto cd = dart::collision::FCLCollisionDetector::create();
    collConstraint = std::make_shared<aikido::constraint::CollisionConstraint>(
        stateSpace, cd);

    // Distance metric
    dmetric = aikido::distance::createDistanceMetricFor(stateSpace);

    // Sampler
    sampler =
        aikido::constraint::createSampleableBounds(stateSpace, make_rng());

    // Projectable constraint
    boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);

    // Joint limits
    boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  }

  void setStateValue(const Eigen::Vector2d &value,
                     aikido::statespace::StateSpace::State *state) const
  {
    auto j1Joint = stateSpace->getSubState<SO2StateSpace::State>(state, 0);
    auto j2Joint = stateSpace->getSubState<SO2StateSpace::State>(state, 1);
    j1Joint->setAngle(value[0]);
    j2Joint->setAngle(value[1]);
  }

  Eigen::Vector2d getStateValue(
      aikido::statespace::StateSpace::State *state) const
  {
    auto j1Joint = stateSpace->getSubState<SO2StateSpace::State>(state, 0);
    auto j2Joint = stateSpace->getSubState<SO2StateSpace::State>(state, 1);

    Eigen::Vector2d retVal(j1Joint->getAngle(), j2Joint->getAngle());
    return retVal;
  }

  BodyNodePtr endEffector;
  TSRPtr goalTSR;
  MetaSkeletonStateSpacePtr stateSpace;
  InterpolatorPtr interpolator;
  DistanceMetricPtr dmetric;
  SampleableConstraintPtr sampler;
  ProjectablePtr boundsProjection;
  TestableConstraintPtr boundsConstraint;
  TestableConstraintPtr collConstraint;
  SampleableConstraintPtr goalSampleable;
  TestableConstraintPtr goalTestable;
};

TEST_F(OMPLGoalSetTest, Plan)
{
  auto startState = stateSpace->createState();
  Eigen::Vector2d startPose(0, 0);
  setStateValue(startPose, startState);

  stateSpace->setStateOnMetaSkeleton(startState);

  // Plan
  auto traj = aikido::ompl::planOMPL<ompl::geometric::RRTConnect>(
      startState, goalTestable, goalSampleable, stateSpace, interpolator,
      std::move(collConstraint), std::move(boundsConstraint),
      std::move(dmetric), std::move(sampler), std::move(boundsProjection),
      5.0);


  // Check the first waypoint
  auto s0 = stateSpace->createState();
  traj->evaluate(0, s0);
  EXPECT_TRUE(getStateValue(s0).isApprox(startPose));

  // Check the last waypoint
  traj->evaluate(traj->getDuration(), s0);
  EXPECT_TRUE(goalTestable->isSatisfied(s0));
}
