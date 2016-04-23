#include <gtest/gtest.h>
#include "../eigen_tests.hpp"
#include <aikido/constraint/CartesianProductSampleable.hpp>
#include <aikido/constraint/Sampleable.hpp>
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>
#include <aikido/constraint/uniform/SO2UniformSampler.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/util/RNG.hpp>

using aikido::constraint::CartesianProductSampleable;
using aikido::constraint::SampleablePtr;
using aikido::statespace::CartesianProduct;
using aikido::statespace::SO2;
using aikido::constraint::SO2Sampleable;
using aikido::statespace::Rn;
using aikido::constraint::RnBoxConstraint;
using aikido::util::RNG;
using aikido::util::RNGWrapper;

class CartesianProductSampleableTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    // Subspaces
    rvss = std::make_shared<Rn>(3);
    so2 = std::make_shared<SO2>();

    // Constraints
    auto rng = std::unique_ptr<RNG>(
      new RNGWrapper<std::default_random_engine>(0));
    rvSampler = std::make_shared<RnBoxConstraint>(
      rvss, rng->clone(), Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    so2Sampler = std::make_shared<SO2Sampleable>(
      so2, rng->clone());
    sampleables.push_back(rvSampler);
    sampleables.push_back(so2Sampler);

    cs = std::make_shared<CartesianProduct>(
        std::vector<aikido::statespace::StateSpacePtr>({rvss, so2}));
  }

  std::shared_ptr<CartesianProduct> cs;
  std::vector<SampleablePtr> sampleables;
  std::shared_ptr<Rn> rvss;
  std::shared_ptr<SO2> so2;
  std::shared_ptr<RnBoxConstraint> rvSampler;
  std::shared_ptr<SO2Sampleable> so2Sampler;

};

TEST_F(CartesianProductSampleableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(CartesianProductSampleable(nullptr, sampleables),
               std::invalid_argument);
}


TEST_F(CartesianProductSampleableTest, ConstructorThrowsOnNullConstraints)
{
  std::vector<SampleablePtr> sampleables; 
  sampleables.push_back(nullptr);
  sampleables.push_back(nullptr);

  EXPECT_THROW(CartesianProductSampleable(cs, sampleables),
               std::invalid_argument);
}


TEST_F(CartesianProductSampleableTest, ConstructorThrowsOnUnmatchingStateSpaceConstraintPair)
{
  std::vector<SampleablePtr> sampleables; 
  sampleables.push_back(so2Sampler);
  sampleables.push_back(rvSampler);

  EXPECT_THROW(CartesianProductSampleable(cs, sampleables),
               std::invalid_argument);
}


TEST_F(CartesianProductSampleableTest, GetStateSpaceMatchesConstructorStateSpace)
{
  auto ss = std::make_shared<CartesianProductSampleable>(cs, sampleables);
  auto space = ss->getStateSpace();
  EXPECT_EQ(space, cs);
}


TEST_F(CartesianProductSampleableTest, SampleGeneratorSamplesCorrectValue)
{
  auto ss = std::make_shared<CartesianProductSampleable>(cs, sampleables);
  auto generator = ss->createSampleGenerator();

  auto state = cs->createState();

  for(int i = 0; i < 10; ++i)
  {
    EXPECT_TRUE(generator->canSample());
    EXPECT_TRUE(generator->sample(state));
    EXPECT_TRUE(rvSampler->isSatisfied(state));
  }
}

