#include "OMPLTestHelpers.hpp"
#include <aikido/planner/ompl/StateSampler.hpp>

using aikido::planner::ompl::GeometricStateSpace;
using aikido::planner::ompl::StateSampler;
using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;

class StateSamplerTest : public PlannerTest
{
public:
  virtual void SetUp()
  {
    PlannerTest::SetUp();
    gSpace = std::make_shared<GeometricStateSpace>(
        stateSpace, interpolator, dmetric, sampler, boundsConstraint,
        boundsProjection);
  }
  std::shared_ptr<GeometricStateSpace> gSpace;
};

TEST_F(StateSamplerTest, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(StateSampler(0, std::move(sampler->createSampleGenerator())),
               std::invalid_argument);
}

TEST_F(StateSamplerTest, ThrowsOnNullGenerator)
{
  EXPECT_THROW(StateSampler(gSpace.get(), nullptr), std::invalid_argument);
}

TEST_F(StateSamplerTest, SampleUniformGeneratorCantSample)
{
  StateSampler ssampler(
      gSpace.get(),
      dart::common::make_unique<EmptySampleGenerator>(stateSpace));
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();

  // Ensure we get two different states if we sample twice
  EXPECT_THROW(ssampler.sampleUniform(s1), std::runtime_error);

  gSpace->freeState(s1);
}

TEST_F(StateSamplerTest, SampleUniformGeneratorFailsSample)
{
  StateSampler ssampler(
      gSpace.get(),
      dart::common::make_unique<EmptySampleGenerator>(stateSpace));
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();

  // Ensure we get two different states if we sample twice
  EXPECT_THROW(ssampler.sampleUniform(s1), std::runtime_error);
  gSpace->freeState(s1);
}

TEST_F(StateSamplerTest, SampleUniformValid)
{
  StateSampler ssampler(gSpace.get(),
                        std::move(sampler->createSampleGenerator()));
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  auto s2 = gSpace->allocState()->as<GeometricStateSpace::StateType>();

  // Ensure we get two different states if we sample twice
  ssampler.sampleUniform(s1);
  ssampler.sampleUniform(s2);
  EXPECT_FALSE(getTranslationalState(stateSpace, s1)
                   .isApprox(getTranslationalState(stateSpace, s2)));
  gSpace->freeState(s1);
  gSpace->freeState(s2);
}

TEST_F(StateSamplerTest, SampleUniformNearAlwaysThrows)
{
  StateSampler ssampler(gSpace.get(),
                        std::move(sampler->createSampleGenerator()));
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  auto s2 = gSpace->allocState()->as<GeometricStateSpace::StateType>();

  EXPECT_THROW(ssampler.sampleUniformNear(s1, s2, 0.05), std::runtime_error);

  gSpace->freeState(s1);
  gSpace->freeState(s2);
}

TEST_F(StateSamplerTest, SampleGaussianAlwaysThrows)
{
  StateSampler ssampler(gSpace.get(),
                        std::move(sampler->createSampleGenerator()));
  auto s1 = gSpace->allocState()->as<GeometricStateSpace::StateType>();
  auto s2 = gSpace->allocState()->as<GeometricStateSpace::StateType>();

  EXPECT_THROW(ssampler.sampleGaussian(s1, s2, 0.05), std::runtime_error);

  gSpace->freeState(s1);
  gSpace->freeState(s2);
}
