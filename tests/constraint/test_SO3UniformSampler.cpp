#include <dart/common/StlHelpers.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/uniform/SO3UniformSampler.hpp>
#include <aikido/distance/SO3Angular.hpp>
#include "SampleGeneratorCoverage.hpp"

using aikido::statespace::SO3;
using aikido::constraint::uniform::SO3UniformSampler;
using aikido::constraint::SampleGenerator;
using aikido::distance::SO3Angular;
using aikido::common::RNG;
using aikido::common::RNGWrapper;
using Eigen::Vector3d;

class SO3UniformSamplerTests : public ::testing::Test
{
protected:
  static constexpr std::size_t NUM_SAMPLES = 1000;
  static constexpr std::size_t NUM_AXIS_TARGETS = 5;
  static constexpr double DISTANCE_THRESHOLD = M_PI / NUM_AXIS_TARGETS;

  void SetUp() override
  {
    mStateSpace = std::make_shared<SO3>();
    mDistance = std::make_shared<SO3Angular>(mStateSpace);
    mRng = std::make_unique<RNGWrapper<std::default_random_engine>>(
        0);

    mTargets.clear();

    for (std::size_t i = 0; i < NUM_AXIS_TARGETS; ++i)
      for (std::size_t j = 0; j < NUM_AXIS_TARGETS; ++j)
        for (std::size_t k = 0; k < NUM_AXIS_TARGETS; ++k)
        {
          const auto angle1 = (2. * M_PI * i) / NUM_AXIS_TARGETS;
          const auto angle2 = (2. * M_PI * j) / NUM_AXIS_TARGETS;
          const auto angle3 = (2. * M_PI * k) / NUM_AXIS_TARGETS;
          const auto rotation = Eigen::Quaterniond(
              Eigen::AngleAxisd(angle1, Vector3d::UnitX())
              * Eigen::AngleAxisd(angle2, Vector3d::UnitY())
              * Eigen::AngleAxisd(angle3, Vector3d::UnitZ()));

          mTargets.emplace_back(mStateSpace->createState());
          mTargets.back().setQuaternion(rotation);
        }
  }

  std::unique_ptr<RNG> mRng;
  std::shared_ptr<SO3> mStateSpace;
  std::shared_ptr<SO3Angular> mDistance;
  std::vector<SO3::ScopedState> mTargets;
};

TEST_F(SO3UniformSamplerTests, constructor_StateSpaceIsNull_Throws)
{
  EXPECT_THROW(
      { SO3UniformSampler(nullptr, mRng->clone()); }, std::invalid_argument);
}

TEST_F(SO3UniformSamplerTests, constructor_RNGIsNull_Throws)
{
  EXPECT_THROW(
      { SO3UniformSampler(mStateSpace, nullptr); }, std::invalid_argument);
}

TEST_F(SO3UniformSamplerTests, sampleGenerator)
{
  SO3UniformSampler constraint(mStateSpace, mRng->clone());

  auto generator = constraint.createSampleGenerator();
  ASSERT_TRUE(generator->canSample());
  ASSERT_EQ(SampleGenerator::NO_LIMIT, generator->getNumSamples());

  auto result = SampleGeneratorCoverage(
      *generator,
      *mDistance,
      std::begin(mTargets),
      std::end(mTargets),
      DISTANCE_THRESHOLD,
      NUM_SAMPLES);
  ASSERT_TRUE(result);
}
