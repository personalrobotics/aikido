#include <aikido/constraint/FiniteSampleable.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include "../eigen_tests.hpp"

#include <gtest/gtest.h>

using aikido::statespace::R1;
using aikido::statespace::R2;
using aikido::constraint::FiniteSampleable;
using aikido::constraint::SampleGenerator;
using State = aikido::statespace::StateSpace::State;

TEST(FiniteSampleableTest, ConstructorThrowsOnNullStateSpace)
{
  R1 rvss;
  auto s1 = rvss.createState();
  s1.setValue(aikido::tests::make_vector(5));

  EXPECT_THROW(FiniteSampleable(nullptr, s1), std::invalid_argument);

  std::vector<const aikido::statespace::StateSpace::State*> states;
  states.push_back(s1);
  EXPECT_THROW(FiniteSampleable(nullptr, states), std::invalid_argument);
}

TEST(FiniteSampleableTest, ConstructorThrowsOnNullState)
{
  auto rvss = std::make_shared<R1>();
  R1::State* st = 0;

  EXPECT_THROW(FiniteSampleable(rvss, st), std::invalid_argument);
}

TEST(FiniteSampleableTest, ConstructorThrowsOnEmptyStates)
{
  auto rvss = std::make_shared<R1>();
  std::vector<const aikido::statespace::StateSpace::State*> states;
  EXPECT_THROW(FiniteSampleable(rvss, states), std::invalid_argument);
}

TEST(FiniteSampleableTest, StateSpaceMatch)
{
  auto rvss = std::make_shared<R1>();
  Eigen::VectorXd v = aikido::tests::make_vector(-2);
  auto s1 = rvss->createState();
  s1.setValue(v);
  FiniteSampleable constraint(rvss, s1);
  EXPECT_EQ(rvss, constraint.getStateSpace());
}

TEST(FiniteSampleableTest, SingleSampleGenerator)
{
  // Single-sample.
  Eigen::VectorXd v = aikido::tests::make_vector(-2);

  R1 rvss;
  auto s1 = rvss.createState();
  s1.setValue(v);

  // Single-sample constraint.
  FiniteSampleable constraint(std::make_shared<R1>(rvss), s1);

  // Single-sample-generator.
  std::unique_ptr<SampleGenerator> generator
      = constraint.createSampleGenerator();

  auto state = rvss.createState();

  // Test the generor's behavior.
  EXPECT_TRUE(generator->canSample());
  EXPECT_EQ(generator->getNumSamples(), 1);

  generator->sample(state);
  EXPECT_TRUE(state.getValue().isApprox(v));

  EXPECT_FALSE(generator->canSample());
  EXPECT_EQ(generator->getNumSamples(), 0);
}

TEST(FiniteSampleableTest, FiniteSampleGenerator)
{

  // Finite-samples
  Eigen::Vector2d v1(0, 1);
  Eigen::Vector2d v2(2, 3);

  std::vector<Eigen::Vector2d> expected;
  expected.push_back(v1);
  expected.push_back(v2);

  R2 rvss;
  auto s1 = rvss.createState();
  s1.setValue(v1);

  auto s2 = rvss.createState();
  s2.setValue(v2);

  std::vector<const State*> states;
  states.push_back(s1);
  states.push_back(s2);

  // Finite-sample-constraint.
  FiniteSampleable constraint(std::make_shared<R2>(rvss), states);

  // Finite-sample generator.
  std::unique_ptr<SampleGenerator> generator
      = constraint.createSampleGenerator();

  auto state = rvss.createState();

  // Test generator's behavior.
  for (int i = 0; i < 2; ++i)
  {
    EXPECT_TRUE(generator->canSample());
    EXPECT_EQ(generator->getNumSamples(), 2 - i);

    generator->sample(state);

    EXPECT_TRUE(state.getValue().isApprox(expected[i % 2]));
  }

  EXPECT_FALSE(generator->canSample());
  EXPECT_EQ(generator->getNumSamples(), 0);
}
