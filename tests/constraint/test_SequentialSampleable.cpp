#include <aikido/constraint/SequentialSampleable.hpp>

#include <Eigen/StdVector>
#include <aikido/constraint/FiniteSampleable.hpp>
#include <aikido/constraint/CyclicSampleable.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include "../eigen_tests.hpp"

#include <gtest/gtest.h>

using aikido::statespace::R1;
using aikido::statespace::R2;
using aikido::constraint::SequentialSampleable;
using aikido::constraint::SampleGenerator;
using State = aikido::statespace::StateSpace::State;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)

TEST(SequentialSampleableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_TRUE(true);
}

TEST(SequentialSampleableTest, ConstructorThrowsOnEmptySampleables)
{
  EXPECT_TRUE(true);
}

TEST(SequentialSampleableTest, ConstructorThrowsOnNullSampleable)
{
  EXPECT_TRUE(true);
}

TEST(SequentialSampleableTest, ConstructorThrowsOnStateSpaceMismatch)
{
  EXPECT_TRUE(true);
}

TEST(SequentialSampleableTest, SingleSampleGenerator)
{
  // Test generator characterstics
  EXPECT_TRUE(true);
}

TEST(SequentialSampleableTest, MultipleSampleGenerators)
{
  // Test generator characterstics
  EXPECT_TRUE(true);
}
