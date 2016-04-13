#include <gtest/gtest.h>
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>
#include <dart/common/StlHelpers.h>
#include <aikido/distance/AngularDistanceMetric.hpp>

using aikido::statespace::SO2StateSpace;
using aikido::statespace::SO2StateSpaceSampleableConstraint;
using aikido::constraint::SampleGenerator;
using aikido::util::RNG;
using aikido::util::RNGWrapper;
using aikido::distance::AngularDistanceMetric;
using dart::common::make_unique;

class SO2UniformSamplerTests : public ::testing::Test
{
protected:
  static constexpr size_t NUM_SAMPLES = 10000;
  static constexpr size_t NUM_TARGETS = 20;
  static constexpr double DISTANCE_THRESHOLD = M_PI / NUM_TARGETS;

  void SetUp() override
  {
    mStateSpace = std::make_shared<SO2StateSpace>();
    mDistance = std::make_shared<AngularDistanceMetric>(mStateSpace);
    mRng = make_unique<RNGWrapper<std::default_random_engine>>(0);

    mAngles.clear();
    for (int i = 0; i < NUM_TARGETS; ++i)
    {
      const double angle = (2 * M_PI * i) / NUM_TARGETS;

      mAngles.emplace_back(mStateSpace->createState());
      mAngles.back().setAngle(angle);
    }
  }

  std::unique_ptr<RNG> mRng;
  std::shared_ptr<SO2StateSpace> mStateSpace;
  std::shared_ptr<AngularDistanceMetric> mDistance;
  std::vector<SO2StateSpace::ScopedState> mAngles;
};

TEST_F(SO2UniformSamplerTests, constructor_StateSpaceIsNull_Throws)
{
  EXPECT_THROW({
    SO2StateSpaceSampleableConstraint(nullptr, mRng->clone());
  }, std::invalid_argument);
}

TEST_F(SO2UniformSamplerTests, constructor_RNGIsNull_Throws)
{
  EXPECT_THROW({
    SO2StateSpaceSampleableConstraint(mStateSpace, nullptr);
  }, std::invalid_argument);
}

TEST_F(SO2UniformSamplerTests, getStateSpace)
{
  SO2StateSpaceSampleableConstraint constraint(mStateSpace, mRng->clone());
  EXPECT_EQ(mStateSpace, constraint.getStateSpace());
}

TEST_F(SO2UniformSamplerTests, createSampleGenerator)
{
  SO2StateSpaceSampleableConstraint constraint(mStateSpace, mRng->clone());
  auto generator = constraint.createSampleGenerator();

  ASSERT_TRUE(!!generator);
  EXPECT_EQ(mStateSpace, generator->getStateSpace());

  // Verify that there are samples "near" each target point.
  auto state = mStateSpace->createState();
  std::vector<int> targetCounts(NUM_TARGETS, 0);

  for (size_t isample = 0; isample < NUM_SAMPLES; ++isample)
  {
    ASSERT_EQ(SampleGenerator::NO_LIMIT, generator->getNumSamples());
    ASSERT_TRUE(generator->canSample());
    ASSERT_TRUE(generator->sample(state));

    for (size_t itarget = 0; itarget < NUM_TARGETS; ++itarget)
    {
      if (mDistance->distance(state, mAngles[itarget]) < DISTANCE_THRESHOLD)
        targetCounts[itarget]++;
    }
  }

  for (auto count : targetCounts)
    ASSERT_GT(count, 0);
}
