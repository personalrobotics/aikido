#include <random>
#include <aikido/constraint/FrameTestable.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/constraint/NonColliding.hpp>
#include <aikido/constraint/dart.hpp>
#include <aikido/constraint/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/FrameTestable.hpp>
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>
#include <aikido/distance/DistanceMetric.hpp>
#include <aikido/distance/Defaults.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/util/RNG.hpp>
#include <dart/dart.h>
#include <gtest/gtest.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <Eigen/Dense>

using aikido::constraint::FrameTestable;
using aikido::constraint::InverseKinematicsSampleable;
using aikido::constraint::ProjectablePtr;
using aikido::constraint::SampleablePtr;
using aikido::constraint::TSR;
using aikido::constraint::TSRPtr;
using aikido::constraint::TestablePtr;
using aikido::distance::DistanceMetricPtr;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::InterpolatorPtr;
using aikido::statespace::SO2;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::util::RNG;
using aikido::util::RNGWrapper;
using dart::common::make_unique;
using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;


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
    goalSampleable = std::make_shared<InverseKinematicsSampleable>(
        stateSpace, goalTSR, std::move(seedSampler), ik, make_rng(), 30);
    goalTestable =
        std::make_shared<FrameTestable>(stateSpace, endEffector.get(), goalTSR);

    // Collision constraint
    auto cd = dart::collision::FCLCollisionDetector::create();
    collConstraint = std::make_shared<aikido::constraint::NonColliding>(
        stateSpace, cd);

    // Distance metric
    dmetric = aikido::distance::createDistanceMetric(stateSpace);

    // Sampler
    sampler =
        aikido::constraint::createSampleableBounds(stateSpace, make_rng());

    // Projectable constraint
    boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);

    // Joint limits
    boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  }

  void setStateValue(const Eigen::Vector2d &value,
                     MetaSkeletonStateSpace::State *state) const
  {
    auto j1Joint = stateSpace->getSubState<SO2::State>(state, 0);
    auto j2Joint = stateSpace->getSubState<SO2::State>(state, 1);
    j1Joint->setAngle(value[0]);
    j2Joint->setAngle(value[1]);
  }

  Eigen::Vector2d getStateValue(MetaSkeletonStateSpace::State *state) const
  {
    auto j1Joint = stateSpace->getSubState<SO2::State>(state, 0);
    auto j2Joint = stateSpace->getSubState<SO2::State>(state, 1);

    Eigen::Vector2d retVal(j1Joint->getAngle(), j2Joint->getAngle());
    return retVal;
  }

  BodyNodePtr endEffector;
  TSRPtr goalTSR;
  MetaSkeletonStateSpacePtr stateSpace;
  InterpolatorPtr interpolator;
  DistanceMetricPtr dmetric;
  SampleablePtr sampler;
  ProjectablePtr boundsProjection;
  TestablePtr boundsConstraint;
  TestablePtr collConstraint;
  SampleablePtr goalSampleable;
  TestablePtr goalTestable;
};
