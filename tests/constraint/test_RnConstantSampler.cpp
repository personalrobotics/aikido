#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <aikido/constraint/uniform/RnConstantSampler.hpp>
#include <aikido/constraint/uniform/RnConstantSampler.hpp>

using namespace aikido;

//==============================================================================
template <int Dimension>
void testConstructorThrowsForNullStateSpace()
{
  EXPECT_THROW({
    constraint::RnConstantSampler(nullptr, Eigen::VectorXd());
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
template <int Dimension>
void testConstructorThrowsForWrongSizeValue()
{
  EXPECT_THROW({
    constraint::RnConstantSampler(
        std::make_shared<statespace::Rn>(Dimension),
        Eigen::VectorXd(Dimension+1));
  }, std::invalid_argument);
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
template <int Dimension>
void testSampleGenerator()
{
  Eigen::VectorXd value = Eigen::VectorXd::Zero(Dimension);
  auto stateSpace = std::make_shared<statespace::Rn>(Dimension);
  EXPECT_EQ(stateSpace->getDimension(), Dimension);

  auto sampler
      = std::make_shared<constraint::RnConstantSampler>(stateSpace, value);
  EXPECT_EQ(sampler->getStateSpace(), stateSpace);

  auto generator = sampler->createSampleGenerator();
  EXPECT_EQ(generator->getStateSpace(), stateSpace);

  for (auto i = 0u; i < 1e+3; ++i)
  {
    EXPECT_TRUE(generator->canSample());

    auto state = stateSpace->createState();
    EXPECT_TRUE(generator->sample(state));

    EXPECT_EQ(state.getValue(), value);
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
