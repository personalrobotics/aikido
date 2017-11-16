#include <stdexcept>
#include <gtest/gtest.h>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include "MockConstraints.hpp"

using aikido::constraint::TestableIntersection;
using aikido::constraint::Testable;
using aikido::statespace::R0;

TEST(ConjuntionConstraintTest, ThrowOnNullStateSpace)
{
  auto ss = std::make_shared<R0>();
  auto pc = std::make_shared<PassingConstraint>(ss);
  EXPECT_THROW(
      TestableIntersection(
          nullptr, std::vector<std::shared_ptr<Testable>>({pc})),
      std::invalid_argument);
}

TEST(TestableIntersectionTest, ReturnCorrectStateSpace)
{
  auto ss = std::make_shared<R0>();
  auto pc = std::make_shared<PassingConstraint>(ss);
  TestableIntersection satisfiedConstraint{
      ss, std::vector<std::shared_ptr<Testable>>({pc, pc})};
  EXPECT_EQ(ss, satisfiedConstraint.getStateSpace());
}

TEST(TestableIntersectionTest, IsSatisfiedReturnsConjuctionOfAllConstraints)
{
  auto ss = std::make_shared<R0>();
  auto pc = std::make_shared<PassingConstraint>(ss);
  auto fc = std::make_shared<FailingConstraint>(ss);

  TestableIntersection satisfiedConstraint{
      ss, std::vector<std::shared_ptr<Testable>>({pc, pc})};
  EXPECT_TRUE(satisfiedConstraint.isSatisfied(nullptr));

  TestableIntersection unsatisfiedConstraint1{
      ss, std::vector<std::shared_ptr<Testable>>({pc, fc})};
  EXPECT_FALSE(unsatisfiedConstraint1.isSatisfied(nullptr));

  TestableIntersection unsatisfiedConstraint2{
      ss, std::vector<std::shared_ptr<Testable>>({fc, pc})};
  EXPECT_FALSE(unsatisfiedConstraint2.isSatisfied(nullptr));

  TestableIntersection unsatisfiedConstraint3{
      ss, std::vector<std::shared_ptr<Testable>>({fc, fc})};
  EXPECT_FALSE(unsatisfiedConstraint3.isSatisfied(nullptr));
}

TEST(TestableIntersectionTest, ReturnsTrueIfNoConstraints)
{
  auto ss = std::make_shared<R0>();
  TestableIntersection cc{ss};
  EXPECT_TRUE(cc.isSatisfied(nullptr));
}

TEST(TestableIntersectionTest, AddConstraintAppendsInitialConstraints)
{
  auto ss = std::make_shared<R0>();
  auto pc = std::make_shared<PassingConstraint>(ss);
  auto fc = std::make_shared<FailingConstraint>(ss);
  TestableIntersection originallyPassingC{
      ss, std::vector<std::shared_ptr<Testable>>({pc})};
  EXPECT_TRUE(originallyPassingC.isSatisfied(nullptr));

  originallyPassingC.addConstraint(fc);
  EXPECT_FALSE(originallyPassingC.isSatisfied(nullptr));

  TestableIntersection stillFailingC{
      ss, std::vector<std::shared_ptr<Testable>>({fc})};

  stillFailingC.addConstraint(pc);
  EXPECT_FALSE(stillFailingC.isSatisfied(nullptr));
}

TEST(TestableIntersectionTest, ThrowIfDifferentStateSpacesOnConstruction)
{
  auto ss1 = std::make_shared<R0>();
  auto ss2 = std::make_shared<aikido::statespace::SO2>();
  auto ss2C = std::make_shared<PassingConstraint>(ss2);
  auto avoidMacro = [&]() {
    TestableIntersection cc{ss1,
                            std::vector<std::shared_ptr<Testable>>({ss2C})};
    return cc;
  };
  EXPECT_THROW(avoidMacro(), std::invalid_argument);
}

TEST(TestableIntersectionTest, ThrowIfDifferentStateSpaceConstraintAdded)
{
  auto ss1 = std::make_shared<R0>();
  auto ss2 = std::make_shared<aikido::statespace::SO2>();
  auto ss2C = std::make_shared<PassingConstraint>(ss2);
  TestableIntersection cc{ss1};
  EXPECT_THROW(cc.addConstraint(ss2C), std::invalid_argument);
}
