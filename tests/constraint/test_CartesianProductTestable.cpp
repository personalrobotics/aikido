#include <gtest/gtest.h>
#include "../eigen_tests.hpp"
#include <aikido/constraint/CartesianProductTestable.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/uniform/RnBoxConstraint.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/Rn.hpp>

using aikido::constraint::CartesianProductTestable;
using aikido::constraint::Satisfied;
using aikido::statespace::CartesianProduct;
using aikido::statespace::SO2;
using aikido::statespace::Rn;
using aikido::constraint::TestablePtr;
using aikido::constraint::Testable;
using aikido::constraint::RnBoxConstraint;

class CartesianProductTestableTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    // Subspaces
    rvss = std::make_shared<Rn>(3);
    so2 = std::make_shared<SO2>();

    // Constraints
    auto rvBox = std::make_shared<RnBoxConstraint>(
      rvss, nullptr, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    satisfied = std::make_shared<Satisfied>(so2);

    testables.push_back(rvBox);
    testables.push_back(satisfied);

    cs = std::make_shared<CartesianProduct>(
        std::vector<aikido::statespace::StateSpacePtr>({rvss, so2}));
    ts = std::make_shared<CartesianProductTestable>(cs, testables);
  }

  std::shared_ptr<CartesianProduct> cs;
  std::shared_ptr<CartesianProductTestable> ts;
  std::vector<TestablePtr> testables;
  std::shared_ptr<Satisfied> satisfied;
  std::shared_ptr<Rn> rvss;
  std::shared_ptr<SO2> so2;

};

TEST_F(CartesianProductTestableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(CartesianProductTestable(nullptr, testables),
               std::invalid_argument);
}

TEST_F(CartesianProductTestableTest, ConstructorThrowsOnNullTestables)
{
  std::vector<TestablePtr> testables;
  testables.push_back(nullptr);
  testables.push_back(nullptr);

  EXPECT_THROW(CartesianProductTestable(cs, testables),
               std::invalid_argument);
}

TEST_F(CartesianProductTestableTest, ConstructorThrowsOnUnmatchingStateSpaceTestablesPair)
{
  auto space = std::make_shared<CartesianProduct>(
        std::vector<aikido::statespace::StateSpacePtr>({rvss, so2, rvss}));
  EXPECT_THROW(CartesianProductTestable(space, testables),
               std::invalid_argument);

  testables.push_back(satisfied);
  EXPECT_THROW(CartesianProductTestable(space, testables),
               std::invalid_argument);
}

TEST_F(CartesianProductTestableTest, GetStateSpaceMatchesConstructorStateSpace)
{
  auto space = ts->getStateSpace();
  EXPECT_EQ(space, cs);
}

TEST_F(CartesianProductTestableTest, IsSastisfiedReturnsTrue)
{
  auto state = cs->createState();

  EXPECT_TRUE(ts->isSatisfied(state));
}


TEST_F(CartesianProductTestableTest, IsSastisfiedReturnsFalse)
{
  auto state = cs->createState();
  auto subState = cs->getSubStateHandle<Rn>(state, 0);

  subState.setValue(Eigen::Vector3d(0, -1, 0));

  EXPECT_FALSE(ts->isSatisfied(state));
}

