#include <aikido/constraint/FiniteCyclicSampleConstraint.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/StateSpace.hpp>

#include <gtest/gtest.h>

using aikido::statespace::RealVectorStateSpace;
using aikido::constraint::FiniteCyclicSampleConstraint;
using aikido::constraint::SampleGenerator;
using State = aikido::statespace::StateSpace::State;


TEST(FiniteCyclicSampleConstraint, SingleState)
{
  Eigen::VectorXd v(1);
  v(0) = -2;

  RealVectorStateSpace rvss(1);
  auto s1 = rvss.createState();
  s1.setValue(v);

  FiniteCyclicSampleConstraint constraint(
    std::make_shared<RealVectorStateSpace>(rvss), s1);

  std::unique_ptr<SampleGenerator> generator = constraint.createSampleGenerator();

  auto state = rvss.createState();

  for(int i = 0; i < 10; ++i)
  {
    EXPECT_TRUE(generator->canSample());
    EXPECT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);

    generator->sample(state);
    EXPECT_TRUE(state.getValue().isApprox(v));
  }
}


TEST(FiniteCyclicSampleConstraint, MultipleStates)
{
  Eigen::Vector2d v1(0, 1);
  Eigen::Vector2d v2(2, 3);
  
  std::vector<Eigen::Vector2d> expected;
  expected.push_back(v1);
  expected.push_back(v2);

  RealVectorStateSpace rvss(2);
  auto s1 = rvss.createState();
  s1.setValue(v1);

  auto s2 = rvss.createState();
  s2.setValue(v2);

  std::vector<const State*> states;
  states.push_back(s1);
  states.push_back(s2);

  FiniteCyclicSampleConstraint constraint(
    std::make_shared<RealVectorStateSpace>(rvss), states);

  std::unique_ptr<SampleGenerator> generator = constraint.createSampleGenerator();

  auto state = rvss.createState();

  for (int i = 0; i < 10; ++i)
  {
    EXPECT_TRUE(generator->canSample());
    EXPECT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);

    generator->sample(state);

    EXPECT_TRUE(state.getValue().isApprox(expected[i%2]));
  }

  EXPECT_TRUE(generator->canSample());
  EXPECT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);

}
