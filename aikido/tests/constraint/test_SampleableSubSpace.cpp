#include <gtest/gtest.h>
#include "../eigen_tests.hpp"
#include <aikido/constraint/SampleableSubSpace.h>
#include <aikido/constraint/Sampleable.hpp>
#include <aikido/constraint/uniform/RealVectorBoxConstraint.hpp>
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/util/RNG.hpp>

using aikido::constraint::SampleableSubSpace;
using aikido::constraint::SampleableConstraintPtr;
using aikido::statespace::CompoundStateSpace;
using aikido::statespace::SO2StateSpace;
using aikido::statespace::SO2StateSpaceSampleableConstraint;
using aikido::statespace::RealVectorStateSpace;
using aikido::statespace::RealVectorBoxConstraint;
using aikido::util::RNG;
using aikido::util::RNGWrapper;

class SampleableSubSpaceTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    // Subspaces
    rvss = std::make_shared<RealVectorStateSpace>(3);
    so2 = std::make_shared<SO2StateSpace>();

    // Constraints
    auto rng = std::unique_ptr<RNG>(
      new RNGWrapper<std::default_random_engine>(0));
    rvSampler = std::make_shared<RealVectorBoxConstraint>(
      rvss, rng->clone(), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    so2Sampler = std::make_shared<SO2StateSpaceSampleableConstraint>(
      so2, rng->clone());
    sampleables.push_back(rvSampler);
    sampleables.push_back(so2Sampler);

    cs = std::make_shared<CompoundStateSpace>(
        std::vector<aikido::statespace::StateSpacePtr>({rvss, so2}));
  }

  std::shared_ptr<CompoundStateSpace> cs;
  std::vector<SampleableConstraintPtr> sampleables;
  std::shared_ptr<RealVectorStateSpace> rvss;
  std::shared_ptr<SO2StateSpace> so2;
  std::shared_ptr<RealVectorBoxConstraint> rvSampler;
  std::shared_ptr<SO2StateSpaceSampleableConstraint> so2Sampler;

};

TEST_F(SampleableSubSpaceTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(SampleableSubSpace(nullptr, sampleables),
               std::invalid_argument);
}


TEST_F(SampleableSubSpaceTest, ConstructorThrowsOnNullConstraints)
{
  std::vector<SampleableConstraintPtr> sampleables; 
  sampleables.push_back(nullptr);
  sampleables.push_back(nullptr);

  EXPECT_THROW(SampleableSubSpace(cs, sampleables),
               std::invalid_argument);
}


TEST_F(SampleableSubSpaceTest, ConstructorThrowsOnUnmatchingStateSpaceConstraintPair)
{
  std::vector<SampleableConstraintPtr> sampleables; 
  sampleables.push_back(so2Sampler);
  sampleables.push_back(rvSampler);

  EXPECT_THROW(SampleableSubSpace(cs, sampleables),
               std::invalid_argument);
}


TEST_F(SampleableSubSpaceTest, GetStateSpaceMatchesConstructorStateSpace)
{
  auto ss = std::make_shared<SampleableSubSpace>(cs, sampleables);
  auto space = ss->getStateSpace();
  EXPECT_EQ(space, cs);
}


TEST_F(SampleableSubSpaceTest, SampleGeneratorSamplesCorrectValue)
{
  auto ss = std::make_shared<SampleableSubSpace>(cs, sampleables);
  auto generator = ss->createSampleGenerator();

  auto state = cs->createState();

  for(int i = 0; i < 10; ++i)
  {
    EXPECT_TRUE(generator->canSample());
    EXPECT_TRUE(generator->sample(state));
    EXPECT_TRUE(rvSampler->isSatisfied(state));
  }
}

