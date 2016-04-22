#include <aikido/constraint/FiniteCyclicSampleConstraint.hpp>
#include <aikido/constraint/FiniteSampleConstraint.hpp>
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/StateSpace.hpp>

#include <gtest/gtest.h>
#include <dart/dart.h>


using aikido::statespace::SO2;
using aikido::statespace::SO2SampleableConstraint;
using aikido::statespace::Rn;
using aikido::constraint::FiniteCyclicSampleConstraint;
using aikido::constraint::FiniteSampleConstraint;
using aikido::constraint::SampleGenerator;
using State = aikido::statespace::StateSpace::State;
using dart::common::make_unique;
using aikido::util::RNGWrapper;
using aikido::util::RNG;
using DefaultRNG = RNGWrapper<std::default_random_engine>;

static std::unique_ptr<DefaultRNG> make_rng()
{
  return make_unique<RNGWrapper<std::default_random_engine>>(0);
}

TEST(FiniteCyclicSampleConstraintTest, ConstructorThrowsOnNullConstraint)
{
  EXPECT_THROW(FiniteCyclicSampleConstraint(nullptr), std::invalid_argument);
}

TEST(FiniteCyclicSampleConstraintTest, ConstructorThrowsOnUnlimitiedSampleGenerator)
{
    auto so2 = std::make_shared<SO2>();
    auto constraint = std::make_shared<SO2SampleableConstraint>(so2, make_rng());
    EXPECT_THROW(std::make_shared<FiniteCyclicSampleConstraint>(constraint),
                 std::invalid_argument);
}

TEST(FiniteCyclicSampleConstraintTest, SingleState)
{
  // Single-sample-state.
  Eigen::VectorXd v(1);
  v(0) = -2;

  Rn rvss(1);
  auto s1 = rvss.createState();
  s1.setValue(v);

  // Single-sample-constraint.
  std::shared_ptr<FiniteSampleConstraint> constraint = std::make_shared<FiniteSampleConstraint>(
    std::make_shared<Rn>(rvss), s1);

  // Single-sample-cyclic-constraint.
  FiniteCyclicSampleConstraint cyclicConstraint(constraint);

  auto generator = cyclicConstraint.createSampleGenerator();

  auto state = rvss.createState();

  // Test consistency of the generator's behavior. 
  for(int i = 0; i < 10; ++i)
  {
    EXPECT_TRUE(generator->canSample());
    EXPECT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);

    generator->sample(state);
    EXPECT_TRUE(state.getValue().isApprox(v));
  }
}

TEST(FiniteCyclicSampleConstraintTest, MultipleStates)
{
  // Finite samples.
  Eigen::Vector2d v1(0, 1);
  Eigen::Vector2d v2(2, 3);
  
  std::vector<Eigen::Vector2d> expected;
  expected.push_back(v1);
  expected.push_back(v2);

  Rn rvss(2);
  auto s1 = rvss.createState();
  s1.setValue(v1);

  auto s2 = rvss.createState();
  s2.setValue(v2);

  std::vector<const State*> states;
  states.push_back(s1);
  states.push_back(s2);

  // Finite-sample constraint
  std::shared_ptr<FiniteSampleConstraint> constraint = 
    std::make_shared<FiniteSampleConstraint>(
    std::make_shared<Rn>(rvss), states);

  // Finite-sample cyclic constraint
  FiniteCyclicSampleConstraint cyclicConstraint(constraint);

  auto generator = cyclicConstraint.createSampleGenerator();

  auto state = rvss.createState();

  // Iterate 10 times and compare the sample value with expected.
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_TRUE(generator->canSample());
    EXPECT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);

    EXPECT_TRUE(generator->sample(state));
    EXPECT_TRUE(state.getValue().isApprox(expected[i%2]));
  }

  EXPECT_TRUE(generator->canSample());
  EXPECT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);

}
