#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <aikido/constraint/uniform/RnConstantSampler.hpp>
#include <aikido/constraint/uniform/RnConstantSampler.hpp>
#include "eigen_tests.hpp"

using namespace aikido;

//==============================================================================
template <int N>
void testConstructorThrowsForNullStateSpace()
{
  EXPECT_THROW({
    constraint::RnConstantSampler<N>(nullptr, Eigen::Matrix<double, N, 1>());
  }, std::invalid_argument);
}

//==============================================================================
TEST(RnConstantSamplerTests, ConstructorThrowsForNullStateSpace)
{
  testConstructorThrowsForNullStateSpace<0>();
  testConstructorThrowsForNullStateSpace<1>();
  testConstructorThrowsForNullStateSpace<2>();
  testConstructorThrowsForNullStateSpace<3>();
  testConstructorThrowsForNullStateSpace<6>();
}

//==============================================================================
template <int N>
void testConstructorThrowsForWrongSizeValue()
{
#ifndef NDEBUG // debug mode
  EXPECT_DEATH({
    constraint::RnConstantSampler<N>(
        std::make_shared<statespace::R<N>>(),
        Eigen::VectorXd::Zero(N+1));
  }, ".*Invalid sizes when resizing a matrix or array..*");
#endif
}

//==============================================================================
TEST(RnConstantSamplerTests, ConstructorThrowsForWrongSizeValue)
{
  testConstructorThrowsForWrongSizeValue<0>();
  testConstructorThrowsForWrongSizeValue<1>();
  testConstructorThrowsForWrongSizeValue<2>();
  testConstructorThrowsForWrongSizeValue<3>();
  testConstructorThrowsForWrongSizeValue<6>();
}

//==============================================================================
template <int N>
void testSampleGenerator()
{
  auto value = Eigen::Matrix<double, N, 1>::Zero();
  EXPECT_EQ(value.size(), N);

  auto stateSpace = std::make_shared<statespace::R<N>>();
  EXPECT_EQ(stateSpace->getDimension(), N);

  auto sampler
      = std::make_shared<constraint::RnConstantSampler<N>>(stateSpace, value);
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
}
