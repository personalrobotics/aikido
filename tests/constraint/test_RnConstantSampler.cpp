#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/uniform/RnConstantSampler.hpp>
#include "eigen_tests.hpp"

using namespace aikido;

//==============================================================================
template <int N>
void testConstructorThrowsForNullStateSpace()
{
  EXPECT_THROW(
      {
        constraint::uniform::RConstantSampler<N>(
            nullptr, Eigen::Matrix<double, N, 1>());
      },
      std::invalid_argument);
}

//==============================================================================
TEST(RnConstantSamplerTests, ConstructorThrowsForNullStateSpace)
{
  testConstructorThrowsForNullStateSpace<0>();
  testConstructorThrowsForNullStateSpace<1>();
  testConstructorThrowsForNullStateSpace<2>();
  testConstructorThrowsForNullStateSpace<3>();
  testConstructorThrowsForNullStateSpace<6>();
  testConstructorThrowsForNullStateSpace<Eigen::Dynamic>();
}

//==============================================================================
template <int N>
void testConstructorThrowsForWrongSizeValue()
{
#ifndef NDEBUG // debug mode
  if (N != Eigen::Dynamic)
  {
    EXPECT_DEATH(
        {
          constraint::uniform::RConstantSampler<N>(
              std::make_shared<statespace::R<N>>(),
              Eigen::VectorXd::Zero(N + 1));
        },
        ".*Invalid sizes when resizing a matrix or array..*");
  }
#endif

  if (N == Eigen::Dynamic)
  {
    EXPECT_THROW(
        {
          constraint::uniform::RConstantSampler<N>(
              std::make_shared<statespace::R<N>>(3), Eigen::VectorXd::Zero(4));
        },
        std::invalid_argument);
  }
}

//==============================================================================
TEST(RnConstantSamplerTests, ConstructorThrowsForWrongSizeValue)
{
  testConstructorThrowsForWrongSizeValue<0>();
  testConstructorThrowsForWrongSizeValue<1>();
  testConstructorThrowsForWrongSizeValue<2>();
  testConstructorThrowsForWrongSizeValue<3>();
  testConstructorThrowsForWrongSizeValue<6>();
  testConstructorThrowsForWrongSizeValue<Eigen::Dynamic>();
}

//==============================================================================
template <int N>
void testSampleGenerator(int dimension = N)
{
  Eigen::VectorXd value = Eigen::VectorXd::Random(dimension);
  EXPECT_EQ(value.size(), dimension);

  auto stateSpace = std::make_shared<statespace::R<N>>(dimension);
  EXPECT_EQ(stateSpace->getDimension(), dimension);

  auto sampler = dart::common::make_aligned_shared<
      constraint::uniform::RConstantSampler<N>>(stateSpace, value);
  EXPECT_EQ(sampler->getStateSpace(), stateSpace);
  EXPECT_TRUE(
      tests::CompareEigenMatrices(sampler->getConstantValue(), value, 1e-6));

  auto generator = sampler->createSampleGenerator();
  EXPECT_EQ(generator->getStateSpace(), stateSpace);

  for (auto i = 0u; i < 1e+3; ++i)
  {
    EXPECT_TRUE(generator->canSample());
    EXPECT_EQ(
        generator->getNumSamples(), constraint::SampleGenerator::NO_LIMIT);

    auto state = stateSpace->createState();
    EXPECT_TRUE(generator->sample(state));

    EXPECT_TRUE(tests::CompareEigenMatrices(
        state.getValue(), sampler->getConstantValue(), 1e-6));
  }
}

//==============================================================================
TEST(RnConstantSamplerTests, SampleGenerator)
{
  testSampleGenerator<0>();
  testSampleGenerator<1>();
  testSampleGenerator<2>();
  testSampleGenerator<3>();
  testSampleGenerator<6>();
  testSampleGenerator<Eigen::Dynamic>(3);
}
