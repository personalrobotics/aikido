#include "OMPLTestHelpers.hpp"
#include <aikido/planner/ompl/AIKIDOGeometricStateSpace.hpp>

using aikido::ompl::GeometricStateSpace;
using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;

class GeometricStateSpaceTest : public OMPLPlannerTest
{
public:
  void constructStateSpace()
  {
    gSpace = std::make_shared<GeometricStateSpace>(
        stateSpace, interpolator, dmetric, sampler, boundsConstraint,
        boundsProjection);
  }
  std::shared_ptr<GeometricStateSpace> gSpace;
};

TEST_F(GeometricStateSpaceTest, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(GeometricStateSpace(nullptr, interpolator, dmetric, sampler,
                                   boundsConstraint, boundsProjection),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnNullInterpolator)
{
  EXPECT_THROW(GeometricStateSpace(stateSpace, nullptr, dmetric, sampler,
                                   boundsConstraint, boundsProjection),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnInterpolatorMismatch)
{
  auto sspace = std::make_shared<StateSpace>(robot);
  auto binterpolator =
      std::make_shared<aikido::statespace::GeodesicInterpolator>(sspace);

  EXPECT_THROW(GeometricStateSpace(stateSpace, binterpolator, dmetric, sampler,
                                   boundsConstraint, boundsProjection),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnNullDistanceMetric)
{
  EXPECT_THROW(GeometricStateSpace(stateSpace, interpolator, nullptr, sampler,
                                   boundsConstraint, boundsProjection),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnDistanceMetricMismatch)
{
  auto sspace = std::make_shared<aikido::statespace::SO2>();
  auto bdmetric = aikido::distance::createDistanceMetric(sspace);

  EXPECT_THROW(
      GeometricStateSpace(stateSpace, interpolator, std::move(bdmetric),
                          sampler, boundsConstraint, boundsProjection),
      std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnNullSampler)
{
  EXPECT_THROW(GeometricStateSpace(stateSpace, interpolator, dmetric, nullptr,
                                   boundsConstraint, boundsProjection),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnSamplerMismatch)
{
  auto sspace = std::make_shared<StateSpace>(robot);
  auto bsampler = aikido::constraint::createSampleableBounds(sspace, make_rng());

  EXPECT_THROW(GeometricStateSpace(stateSpace, interpolator, dmetric,
                                   std::move(bsampler), boundsConstraint,
                                   boundsProjection),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnNullBoundsConstraint)
{
  EXPECT_THROW(GeometricStateSpace(stateSpace, interpolator, dmetric, sampler,
                                   nullptr, boundsProjection),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnBoundsConstraintMismatch)
{
  auto sspace = std::make_shared<StateSpace>(robot);
  auto bconstraint = aikido::constraint::createTestableBounds(sspace);

  EXPECT_THROW(GeometricStateSpace(stateSpace, interpolator, dmetric, sampler,
                                   std::move(bconstraint), boundsProjection),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnNullBoundsProjection)
{
  EXPECT_THROW(GeometricStateSpace(stateSpace, interpolator, dmetric, sampler,
                                   boundsConstraint, nullptr),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, ThrowsOnBoundsProjectionMismatch)
{
  auto sspace = std::make_shared<StateSpace>(robot);
  auto bconstraint = aikido::constraint::createProjectableBounds(sspace);

  EXPECT_THROW(GeometricStateSpace(stateSpace, interpolator, dmetric, sampler,
                                   boundsConstraint, std::move(bconstraint)),
               std::invalid_argument);
}

TEST_F(GeometricStateSpaceTest, Dimension)
{
  constructStateSpace();
  EXPECT_EQ(3, gSpace->getDimension());
}

// TODO: Maximum Extent

// TODO: Measure

TEST_F(GeometricStateSpaceTest, EnforceBoundsProjection)
{
  constructStateSpace();

  auto state = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d badValue(-6, 16, 10);
  setTranslationalState(badValue, stateSpace, state);

  gSpace->enforceBounds(state);
  EXPECT_TRUE(getTranslationalState(stateSpace, state)
                  .isApprox(Eigen::Vector3d(-5, 5, 0)));
  gSpace->freeState(state);
}

TEST_F(GeometricStateSpaceTest, EnforceBoundsNoProjection)
{
  constructStateSpace();
  auto state = gSpace->allocState()->as<GeometricStateSpace::StateType>();

  Eigen::Vector3d goodValue(2, -3, 0);
  setTranslationalState(goodValue, stateSpace, state);
  gSpace->enforceBounds(state);
  EXPECT_TRUE(getTranslationalState(stateSpace, state).isApprox(goodValue));

  gSpace->freeState(state);
}

TEST_F(GeometricStateSpaceTest, SatisfiesBoundsFalse) 
{
  constructStateSpace();
  auto state = gSpace->allocState()->as<GeometricStateSpace::StateType>();

  Eigen::Vector3d badValue(-6, 16, 10);
  setTranslationalState(badValue, stateSpace, state);
  EXPECT_FALSE(gSpace->satisfiesBounds(state));

  gSpace->freeState(state);
}
TEST_F(GeometricStateSpaceTest, SatisfiesBoundsTrue)
{
  constructStateSpace();
  auto state = gSpace->allocState()->as<GeometricStateSpace::StateType>();

  Eigen::Vector3d goodValue(2, -3, 0);
  setTranslationalState(goodValue, stateSpace, state);
  EXPECT_TRUE(gSpace->satisfiesBounds(state));

  gSpace->freeState(state);
}

TEST_F(GeometricStateSpaceTest, CopyState)
{
  constructStateSpace();
  auto state = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d value(-2, 3, 0);
  setTranslationalState(value, stateSpace, state);

  auto copyState = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  gSpace->copyState(copyState, state);
  EXPECT_TRUE(getTranslationalState(stateSpace, copyState)
                  .isApprox(getTranslationalState(stateSpace, state)));

  gSpace->freeState(state);
  gSpace->freeState(copyState);
}

TEST_F(GeometricStateSpaceTest, Distance)
{
  constructStateSpace();
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d v1(-2, 3, 0);
  setTranslationalState(v1, stateSpace, s1);

  auto s2 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d v2(3, 4, 0);
  setTranslationalState(v2, stateSpace, s2);

  EXPECT_DOUBLE_EQ((v1 - v2).norm(), gSpace->distance(s1, s2));

  gSpace->freeState(s1);
  gSpace->freeState(s2);
}

TEST_F(GeometricStateSpaceTest, EqualStatesFalse)
{
  constructStateSpace();
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d v1(-2, 3, 0);
  setTranslationalState(v1, stateSpace, s1);

  auto s2 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d v2(2, -3, 0);
  setTranslationalState(v2, stateSpace, s2);

  EXPECT_FALSE(gSpace->equalStates(s1, s2));

  gSpace->freeState(s1);
  gSpace->freeState(s2);
}

TEST_F(GeometricStateSpaceTest, EqualStatesTrue)
{
  constructStateSpace();
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d v1(-2, 3, 0);
  setTranslationalState(v1, stateSpace, s1);

  auto s2 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d v2(-2, 3, 0);
  setTranslationalState(v2, stateSpace, s2);

  EXPECT_TRUE(gSpace->equalStates(s1, s2));

  gSpace->freeState(s1);
  gSpace->freeState(s2);

}

TEST_F(GeometricStateSpaceTest, Interpolate)
{
  constructStateSpace();
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d v1(-2, 3, 0);
  setTranslationalState(v1, stateSpace, s1);

  auto s2 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d v2(3, 4, 0);
  setTranslationalState(v2, stateSpace, s2);

  auto s3 = gSpace->allocState()->as<GeometricStateSpace::StateType>();

  gSpace->interpolate(s1, s2, 0, s3);
  EXPECT_TRUE(getTranslationalState(stateSpace, s3)
                  .isApprox(getTranslationalState(stateSpace, s1)));

  gSpace->interpolate(s1, s2, 1, s3);
  EXPECT_TRUE(getTranslationalState(stateSpace, s3)
                  .isApprox(getTranslationalState(stateSpace, s2)));

  gSpace->interpolate(s1, s2, 0.5, s3);
  EXPECT_TRUE(getTranslationalState(stateSpace, s3)
                  .isApprox(Eigen::Vector3d(0.5, 3.5, 0)));

  gSpace->freeState(s1);
  gSpace->freeState(s2);
  gSpace->freeState(s3);
}

TEST_F(GeometricStateSpaceTest, AllocStateSampler)
{
  constructStateSpace();
  ompl::base::StateSamplerPtr ssampler = gSpace->allocDefaultStateSampler();
  EXPECT_FALSE(ssampler == nullptr);
}

TEST_F(GeometricStateSpaceTest, CopyAlloc)
{
  constructStateSpace();
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  Eigen::Vector3d value(-2, 3, 0);
  setTranslationalState(value, stateSpace, s1);

  auto s2 =
  gSpace->allocState(s1->mState)->as<GeometricStateSpace::StateType>();
  EXPECT_TRUE(getTranslationalState(stateSpace, s1)
                  .isApprox(getTranslationalState(stateSpace, s2)));

  gSpace->freeState(s1);
  gSpace->freeState(s2);
}
