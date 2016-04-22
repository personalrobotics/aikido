#include "OMPLTestHelpers.hpp"
#include "../../constraint/MockConstraints.hpp"
#include <aikido/planner/ompl/AIKIDOStateValidityChecker.hpp>
#include <aikido/planner/ompl/OMPLPlanner.hpp>

using aikido::planner::ompl::GeometricStateSpace;
using aikido::planner::ompl::StateValidityChecker;
using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;

class StateValidityCheckerTest : public OMPLPlannerTest
{
public:
  virtual void SetUp()
  {
    OMPLPlannerTest::SetUp();
    gSpace = std::make_shared<GeometricStateSpace>(
        stateSpace, interpolator, dmetric, sampler, boundsConstraint,
        boundsProjection);
  }
  std::shared_ptr<GeometricStateSpace> gSpace;
};

TEST_F(StateValidityCheckerTest, ThrowsOnNullSpaceInformation)
{
  auto constraint =
      std::make_shared<aikido::constraint::SatisfiedConstraint>(stateSpace);
  EXPECT_THROW(StateValidityChecker(nullptr, std::move(constraint)),
               std::invalid_argument);
}

TEST_F(StateValidityCheckerTest, ThrowsOnNullConstraint)
{
  auto si = aikido::planner::ompl::getSpaceInformation(
      stateSpace, interpolator, dmetric, sampler, collConstraint,
      boundsConstraint, boundsProjection);
  EXPECT_THROW(StateValidityChecker(si, nullptr), std::invalid_argument);
}

TEST_F(StateValidityCheckerTest, ValidState)
{
  auto si = aikido::planner::ompl::getSpaceInformation(
      stateSpace, interpolator, dmetric, sampler, collConstraint,
      boundsConstraint, boundsProjection);
  auto constraint = std::make_shared<PassingConstraint>(stateSpace);
  StateValidityChecker vchecker(si, constraint);
  auto state = si->allocState();
  EXPECT_TRUE(vchecker.isValid(state));
  si->freeState(state);
}

TEST_F(StateValidityCheckerTest, InvalidState)
{
  auto si = aikido::planner::ompl::getSpaceInformation(
      stateSpace, interpolator, dmetric, sampler, collConstraint,
      boundsConstraint, boundsProjection);
  auto constraint = std::make_shared<FailingConstraint>(stateSpace);
  StateValidityChecker vchecker(si, constraint);
  auto state = si->allocState();
  EXPECT_FALSE(vchecker.isValid(state));
  si->freeState(state);
}
