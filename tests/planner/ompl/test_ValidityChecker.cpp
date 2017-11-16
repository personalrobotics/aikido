#include <aikido/planner/ompl/Planner.hpp>
#include <aikido/planner/ompl/StateValidityChecker.hpp>
#include "../../constraint/MockConstraints.hpp"
#include "OMPLTestHelpers.hpp"

using aikido::planner::ompl::GeometricStateSpace;
using aikido::planner::ompl::StateValidityChecker;
using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;

class StateValidityCheckerTest : public PlannerTest
{
public:
  virtual void SetUp()
  {
    PlannerTest::SetUp();
    gSpace = std::make_shared<GeometricStateSpace>(
        stateSpace,
        interpolator,
        dmetric,
        sampler,
        boundsConstraint,
        boundsProjection);
    si = aikido::planner::ompl::getSpaceInformation(
        stateSpace,
        interpolator,
        dmetric,
        sampler,
        collConstraint,
        boundsConstraint,
        boundsProjection,
        0.1);
  }
  std::shared_ptr<GeometricStateSpace> gSpace;
  ::ompl::base::SpaceInformationPtr si;
};

TEST_F(StateValidityCheckerTest, ThrowsOnNullSpaceInformation)
{
  auto constraint = std::make_shared<aikido::constraint::Satisfied>(stateSpace);
  EXPECT_THROW(
      StateValidityChecker(nullptr, std::move(constraint)),
      std::invalid_argument);
}

TEST_F(StateValidityCheckerTest, ThrowsOnNullConstraint)
{
  EXPECT_THROW(StateValidityChecker(si, nullptr), std::invalid_argument);
}

TEST_F(StateValidityCheckerTest, ValidState)
{
  auto constraint = std::make_shared<PassingConstraint>(stateSpace);
  StateValidityChecker vchecker(si, constraint);
  auto state = si->allocState();
  EXPECT_TRUE(vchecker.isValid(state));
  si->freeState(state);
}

TEST_F(StateValidityCheckerTest, InvalidState)
{
  auto constraint = std::make_shared<PassingConstraint>(stateSpace);
  StateValidityChecker vchecker(si, constraint);
  auto state = si->allocState()->as<GeometricStateSpace::StateType>();
  state->mValid = false;
  EXPECT_FALSE(vchecker.isValid(state));
  si->freeState(state);
}

TEST_F(StateValidityCheckerTest, FailedConstraint)
{
  auto constraint = std::make_shared<FailingConstraint>(stateSpace);
  StateValidityChecker vchecker(si, constraint);
  auto state = si->allocState();
  EXPECT_FALSE(vchecker.isValid(state));
  si->freeState(state);
}

TEST_F(StateValidityCheckerTest, NullAikidoState)
{
  auto constraint = std::make_shared<PassingConstraint>(stateSpace);
  StateValidityChecker vchecker(si, constraint);
  auto state = si->allocState()->as<GeometricStateSpace::StateType>();
  stateSpace->freeState(state->mState);
  state->mState = nullptr;
  EXPECT_FALSE(vchecker.isValid(state));
  si->freeState(state);
}

TEST_F(StateValidityCheckerTest, NullOmplState)
{
  auto constraint = std::make_shared<PassingConstraint>(stateSpace);
  StateValidityChecker vchecker(si, constraint);
  EXPECT_FALSE(vchecker.isValid(nullptr));
}
