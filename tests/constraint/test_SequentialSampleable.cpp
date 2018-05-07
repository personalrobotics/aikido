#include <aikido/constraint/SequentialSampleable.hpp>

#include <Eigen/StdVector>
#include <aikido/constraint/CyclicSampleable.hpp>
#include <aikido/constraint/FiniteSampleable.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include "../eigen_tests.hpp"

#include <gtest/gtest.h>

using aikido::statespace::R1;
using aikido::statespace::R2;
using aikido::constraint::FiniteSampleable;
using aikido::constraint::ConstSampleablePtr;
using aikido::constraint::SequentialSampleable;
using aikido::constraint::SampleGenerator;
using State = aikido::statespace::StateSpace::State;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)

TEST(SequentialSampleableTest, ConstructorThrowsOnNullStateSpace)
{
  auto r1 = std::make_shared<R1>();
  auto s1 = r1->createState();
  s1.setValue(aikido::tests::make_vector(-2));

  std::vector<ConstSampleablePtr> sampleables;
  sampleables.emplace_back(std::make_shared<const FiniteSampleable>(r1, s1));

  EXPECT_THROW(
      SequentialSampleable(nullptr, sampleables), std::invalid_argument);
}

TEST(SequentialSampleableTest, ConstructorThrowsOnEmptySampleables)
{
  auto r1 = std::make_shared<R1>();
  auto s1 = r1->createState();
  s1.setValue(aikido::tests::make_vector(-2));

  std::vector<ConstSampleablePtr> sampleables;

  EXPECT_THROW(SequentialSampleable(r1, sampleables), std::invalid_argument);
}

TEST(SequentialSampleableTest, ConstructorThrowsOnNullSampleable)
{
  auto r1 = std::make_shared<R1>();
  auto s1 = r1->createState();
  s1.setValue(aikido::tests::make_vector(-2));

  std::vector<ConstSampleablePtr> sampleables;
  sampleables.emplace_back(nullptr);

  EXPECT_THROW(
      SequentialSampleable(nullptr, sampleables), std::invalid_argument);
}

TEST(SequentialSampleableTest, ConstructorThrowsOnStateSpaceMismatch)
{
  auto r1 = std::make_shared<R1>();
  auto r2 = std::make_shared<R2>();
  auto s1 = r1->createState();
  s1.setValue(aikido::tests::make_vector(-2));

  std::vector<ConstSampleablePtr> sampleables;
  sampleables.emplace_back(std::make_shared<const FiniteSampleable>(r1, s1));

  EXPECT_THROW(SequentialSampleable(r2, sampleables), std::invalid_argument);
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
