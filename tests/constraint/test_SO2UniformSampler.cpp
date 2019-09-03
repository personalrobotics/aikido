#include <gtest/gtest.h>

#include "aikido/common/memory.hpp"
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>
#include <aikido/distance/SO2Angular.hpp>

#include "SampleGeneratorCoverage.hpp"

using aikido::common::RNG;
using aikido::common::RNGWrapper;
using aikido::constraint::SampleGenerator;
using aikido::constraint::uniform::SO2UniformSampler;
using aikido::distance::SO2Angular;
using aikido::statespace::SO2;

class SO2UniformSamplerTests : public ::testing::Test
{
protected:
  static constexpr std::size_t NUM_SAMPLES = 10000;
  static constexpr std::size_t NUM_TARGETS = 20;
  static constexpr double DISTANCE_THRESHOLD = M_PI / NUM_TARGETS;

  void SetUp() override
  {
    mStateSpace = std::make_shared<SO2>();
    mDistance = std::make_shared<SO2Angular>(mStateSpace);
    mRng
        = ::aikido::common::make_unique<RNGWrapper<std::default_random_engine>>(
            0);

    mTargets.clear();
    for (std::size_t i = 0; i < NUM_TARGETS; ++i)
    {
      const double angle = (2 * M_PI * i) / NUM_TARGETS;

      mTargets.emplace_back(mStateSpace->createState());
      mTargets.back().fromAngle(angle);
    }
  }

  std::unique_ptr<RNG> mRng;
  std::shared_ptr<SO2> mStateSpace;
  std::shared_ptr<SO2Angular> mDistance;
  std::vector<SO2::ScopedState> mTargets;
};

TEST_F(SO2UniformSamplerTests, constructor_StateSpaceIsNull_Throws)
{
  EXPECT_THROW(
      { SO2UniformSampler(nullptr, mRng->clone()); }, std::invalid_argument);
}

TEST_F(SO2UniformSamplerTests, constructor_RNGIsNull_Throws)
{
  EXPECT_THROW(
      { SO2UniformSampler(mStateSpace, nullptr); }, std::invalid_argument);
}

TEST_F(SO2UniformSamplerTests, getStateSpace)
{
  SO2UniformSampler constraint(mStateSpace, mRng->clone());
  EXPECT_EQ(mStateSpace, constraint.getStateSpace());
}

TEST_F(SO2UniformSamplerTests, createSampleGenerator)
{
  SO2UniformSampler constraint(mStateSpace, mRng->clone());
  auto generator = constraint.createSampleGenerator();

  ASSERT_TRUE(!!generator);
  EXPECT_EQ(mStateSpace, generator->getStateSpace());

  auto result = SampleGeneratorCoverage(
      *generator,
      *mDistance,
      std::begin(mTargets),
      std::end(mTargets),
      DISTANCE_THRESHOLD,
      NUM_SAMPLES);
  ASSERT_TRUE(result);
}
