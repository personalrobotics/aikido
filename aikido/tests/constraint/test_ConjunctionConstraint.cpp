#include <aikido/constraint/ConjunctionConstraint.hpp>
#include <gtest/gtest.h>
#include "../constraint/MockConstraints.hpp"
#include <stdexcept>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>

using aikido::constraint::ConjunctionConstraint;
using aikido::constraint::TestableConstraint;
using aikido::statespace::RealVectorStateSpace;

TEST(ConjunctionConstraintTest, IsSatisfiedReturnsConjuctionOfAllConstraints)
{
  auto ss = std::make_shared<RealVectorStateSpace>(0);
  auto pc = std::make_shared<PassingConstraint>(ss);
  auto fc = std::make_shared<FailingConstraint>(ss);

  ConjunctionConstraint satisfiedConstraint{
      ss, std::vector<std::shared_ptr<TestableConstraint>>({pc, pc})};
  EXPECT_TRUE(satisfiedConstraint.isSatisfied(nullptr));

  ConjunctionConstraint unsatisfiedConstraint1{
      ss, std::vector<std::shared_ptr<TestableConstraint>>({pc, fc})};
  EXPECT_FALSE(unsatisfiedConstraint1.isSatisfied(nullptr));

  ConjunctionConstraint unsatisfiedConstraint2{
      ss, std::vector<std::shared_ptr<TestableConstraint>>({fc, pc})};
  EXPECT_FALSE(unsatisfiedConstraint2.isSatisfied(nullptr));

  ConjunctionConstraint unsatisfiedConstraint3{
      ss, std::vector<std::shared_ptr<TestableConstraint>>({fc, fc})};
  EXPECT_FALSE(unsatisfiedConstraint3.isSatisfied(nullptr));
}

TEST(ConjunctionConstraintTest, ReturnsTrueIfNoConstraints)
{
  auto ss = std::make_shared<RealVectorStateSpace>(0);
  ConjunctionConstraint cc{ss};
  EXPECT_TRUE(cc.isSatisfied(nullptr));
}

TEST(ConjunctionConstraintTest, AddConstraintAppendsInitialConstraints)
{
  auto ss = std::make_shared<RealVectorStateSpace>(0);
  auto pc = std::make_shared<PassingConstraint>(ss);
  auto fc = std::make_shared<FailingConstraint>(ss);
  ConjunctionConstraint originallyPassingC{
      ss, std::vector<std::shared_ptr<TestableConstraint>>({pc})};

  originallyPassingC.addConstraint(fc);
  EXPECT_FALSE(originallyPassingC.isSatisfied(nullptr));

  ConjunctionConstraint stillFailingC{
      ss, std::vector<std::shared_ptr<TestableConstraint>>({fc})};

  stillFailingC.addConstraint(pc);
  EXPECT_FALSE(stillFailingC.isSatisfied(nullptr));
}

TEST(ConjunctionConstraintTest, ThrowIfDifferentStateSpacesOnConstruction)
{
  auto ss1 = std::make_shared<RealVectorStateSpace>(0);
  auto ss2 = std::make_shared<aikido::statespace::SO2StateSpace>();
  auto ss2C = std::make_shared<PassingConstraint>(ss2);
  auto avoidMacro = [&]() {
    ConjunctionConstraint cc{
        ss1, std::vector<std::shared_ptr<TestableConstraint>>({ss2C})};
    return cc;
  };
  EXPECT_THROW(avoidMacro(), std::invalid_argument);
}

TEST(ConjunctionConstraintTest, ThrowIfDifferentStateSpaceConstraintAdded)
{
  auto ss1 = std::make_shared<RealVectorStateSpace>(0);
  auto ss2 = std::make_shared<aikido::statespace::SO2StateSpace>();
  auto ss2C = std::make_shared<PassingConstraint>(ss2);
  ConjunctionConstraint cc{ss1};
  EXPECT_THROW(cc.addConstraint(ss2C), std::invalid_argument);
}
