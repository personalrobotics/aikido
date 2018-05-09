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
using aikido::constraint::CyclicSampleable;
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

  EXPECT_THROW(SequentialSampleable(r1, sampleables), std::invalid_argument);
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
  auto r1 = std::make_shared<R1>();

  auto s1 = r1->createState();
  Eigen::VectorXd v = aikido::tests::make_vector(-2);
  s1.setValue(v);

  std::vector<ConstSampleablePtr> sampleables;
  sampleables.emplace_back(std::make_shared<const FiniteSampleable>(r1, s1));

  SequentialSampleable constraint(r1, sampleables);

  // Single-generator.
  std::unique_ptr<SampleGenerator> generator
      = constraint.createSampleGenerator();

  auto state = r1->createState();

  // Test the generator's behavior.
  EXPECT_TRUE(generator->canSample());
  EXPECT_EQ(generator->getNumSamples(), 1);

  generator->sample(state);
  EXPECT_TRUE(state.getValue().isApprox(v));

  EXPECT_FALSE(generator->canSample());
  EXPECT_EQ(generator->getNumSamples(), 0);
}

TEST(SequentialSampleableTest, MultipleSampleGenerators)
{
  auto r1 = std::make_shared<R1>();

  auto s1 = r1->createState();
  auto s2 = r1->createState();
  Eigen::VectorXd v1 = aikido::tests::make_vector(-2);
  Eigen::VectorXd v2 = aikido::tests::make_vector(-1);
  s1.setValue(v1);
  s2.setValue(v2);

  std::vector<ConstSampleablePtr> sampleables;
  sampleables.emplace_back(std::make_shared<const FiniteSampleable>(r1, s1));
  sampleables.emplace_back(
      std::make_shared<const CyclicSampleable>(
          std::make_shared<FiniteSampleable>(r1, s2)));

  SequentialSampleable constraint(r1, sampleables);

  // Multiple-generator.
  std::unique_ptr<SampleGenerator> generator
      = constraint.createSampleGenerator();

  auto state = r1->createState();

  // Test the generator's behavior.
  EXPECT_TRUE(generator->canSample());
  EXPECT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);

  generator->sample(state);
  EXPECT_TRUE(state.getValue().isApprox(v1));

  EXPECT_TRUE(generator->canSample());
  EXPECT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);

  generator->sample(state);
  EXPECT_TRUE(state.getValue().isApprox(v2));
}
