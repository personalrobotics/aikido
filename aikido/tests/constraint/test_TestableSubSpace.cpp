#include <gtest/gtest.h>
#include "../eigen_tests.hpp"
#include <aikido/constraint/TestableSubSpace.hpp>
#include <aikido/constraint/TestableConstraint.hpp>
#include <aikido/constraint/uniform/RealVectorBoxConstraint.hpp>
#include <aikido/constraint/SatisfiedConstraint.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>

using aikido::constraint::TestableSubSpace;
using aikido::constraint::SatisfiedConstraint;
using aikido::statespace::CompoundStateSpace;
using aikido::statespace::SO2StateSpace;
using aikido::statespace::RealVectorStateSpace;
using aikido::constraint::TestableConstraintPtr;
using aikido::constraint::TestableConstraint;
using aikido::statespace::RealVectorBoxConstraint;

class TestableSubSpaceTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    // Subspaces
    rvss = std::make_shared<RealVectorStateSpace>(3);
    so2 = std::make_shared<SO2StateSpace>();

    // Constraints
    auto rvBox = std::make_shared<RealVectorBoxConstraint>(
      rvss, nullptr, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
    satisfied = std::make_shared<SatisfiedConstraint>(so2);

    testables.push_back(rvBox);
    testables.push_back(satisfied);

    cs = std::make_shared<CompoundStateSpace>(
        std::vector<aikido::statespace::StateSpacePtr>({rvss, so2}));
    ts = std::make_shared<TestableSubSpace>(cs, testables);
  }

  std::shared_ptr<CompoundStateSpace> cs;
  std::shared_ptr<TestableSubSpace> ts;
  std::vector<TestableConstraintPtr> testables;
  std::shared_ptr<SatisfiedConstraint> satisfied;
  std::shared_ptr<RealVectorStateSpace> rvss;
  std::shared_ptr<SO2StateSpace> so2;

};

TEST_F(TestableSubSpaceTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(TestableSubSpace(nullptr, testables),
               std::invalid_argument);
}

TEST_F(TestableSubSpaceTest, ConstructorThrowsOnNullTestables)
{
  std::vector<TestableConstraintPtr> testables;
  testables.push_back(nullptr);
  testables.push_back(nullptr);

  EXPECT_THROW(TestableSubSpace(cs, testables),
               std::invalid_argument);
}

TEST_F(TestableSubSpaceTest, ConstructorThrowsOnUnmatchingStateSpaceTestablesPair)
{
  auto space = std::make_shared<CompoundStateSpace>(
        std::vector<aikido::statespace::StateSpacePtr>({rvss, so2, rvss}));
  EXPECT_THROW(TestableSubSpace(space, testables),
               std::invalid_argument);

  testables.push_back(satisfied);
  EXPECT_THROW(TestableSubSpace(space, testables),
               std::invalid_argument);
}

TEST_F(TestableSubSpaceTest, GetStateSpaceMatchesConstructorStateSpace)
{
  auto space = ts->getStateSpace();
  EXPECT_EQ(space, cs);
}

TEST_F(TestableSubSpaceTest, IsSastisfiedReturnsTrue)
{
  auto state = cs->createState();

  EXPECT_TRUE(ts->isSatisfied(state));
}


TEST_F(TestableSubSpaceTest, IsSastisfiedReturnsFalse)
{
  auto state = cs->createState();
  auto subState = cs->getSubStateHandle<RealVectorStateSpace>(state, 0);

  subState.setValue(Eigen::Vector3d(0, -1, 0));

  EXPECT_FALSE(ts->isSatisfied(state));
}

