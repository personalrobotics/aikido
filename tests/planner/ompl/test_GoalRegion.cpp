#include <ompl/base/spaces/SO2StateSpace.h>
#include <aikido/planner/ompl/GoalRegion.hpp>
#include <aikido/planner/ompl/Planner.hpp>
#include "../../constraint/MockConstraints.hpp"
#include "OMPLTestHelpers.hpp"

using aikido::planner::ompl::GeometricStateSpace;
using aikido::planner::ompl::GoalRegion;
using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;

class GoalRegionTest : public PlannerTest
{
public:
  virtual void SetUp()
  {
    PlannerTest::SetUp();
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
  ::ompl::base::SpaceInformationPtr si;
};

TEST_F(GoalRegionTest, ThrowsOnNullSpaceInformation)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  EXPECT_THROW(
      GoalRegion(nullptr, std::move(testable), std::move(generator)),
      std::invalid_argument);
}

TEST_F(GoalRegionTest, ThrowsOnNullTestable)
{
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  EXPECT_THROW(
      GoalRegion(si, nullptr, std::move(generator)), std::invalid_argument);
}

TEST_F(GoalRegionTest, ThrowsOnNullGenerator)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  EXPECT_THROW(
      GoalRegion(si, std::move(testable), nullptr), std::invalid_argument);
}

TEST_F(GoalRegionTest, ThrowsOnTestableGeneratorMismatch)
{
  auto so3 = std::make_shared<aikido::statespace::SO3>();
  auto testable = std::make_shared<PassingConstraint>(so3);
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  EXPECT_THROW(
      GoalRegion(si, std::move(testable), std::move(generator)),
      std::invalid_argument);
}

TEST_F(GoalRegionTest, DifferentSample)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  GoalRegion gr(si, std::move(testable), sampler->createSampleGenerator());

  auto state1 = si->allocState();
  auto state2 = si->allocState();

  // Check that successive sample calls generate different states
  gr.sampleGoal(state1);
  gr.sampleGoal(state2);
  EXPECT_FALSE(
      getTranslationalState(stateSpace, state1)
          .isApprox(getTranslationalState(stateSpace, state2)));
  si->freeState(state1);
  si->freeState(state2);
}

TEST_F(GoalRegionTest, ValidSample)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  GoalRegion gr(si, std::move(testable), sampler->createSampleGenerator());

  auto state1 = si->allocState();

  // Check that a sampled state satisfies the goal
  gr.sampleGoal(state1);
  EXPECT_TRUE(gr.isSatisfied(state1));
  si->freeState(state1);
}

TEST_F(GoalRegionTest, CantSample)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));

  auto state = si->allocState()->as<GeometricStateSpace::StateType>();
  gr.sampleGoal(state);
  EXPECT_FALSE(state->mValid);
  si->freeState(state);
}

TEST_F(GoalRegionTest, FailedSample)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<FailedSampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));

  auto state = si->allocState()->as<GeometricStateSpace::StateType>();
  gr.sampleGoal(state);
  EXPECT_FALSE(state->mValid);
  si->freeState(state);
}

TEST_F(GoalRegionTest, NumSamples)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<FailedSampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));
  EXPECT_EQ(1000, gr.maxSampleCount());
}

TEST_F(GoalRegionTest, CouldSampleTrue)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<FailedSampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));
  EXPECT_TRUE(gr.couldSample());
}

TEST_F(GoalRegionTest, CouldSampleFalse)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));
  EXPECT_FALSE(gr.couldSample());
}

TEST_F(GoalRegionTest, ZeroDistance)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));

  auto state = si->allocState();
  EXPECT_DOUBLE_EQ(0, gr.distanceGoal(state));
  si->freeState(state);
}

TEST_F(GoalRegionTest, InfiniteDistance)
{
  auto testable = std::make_shared<FailingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));

  auto state = si->allocState();
  EXPECT_DOUBLE_EQ(
      std::numeric_limits<double>::infinity(), gr.distanceGoal(state));
  si->freeState(state);
}

TEST_F(GoalRegionTest, GoalSatisfied)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));

  auto state = si->allocState();
  EXPECT_TRUE(gr.isSatisfied(state));
  si->freeState(state);
}

TEST_F(GoalRegionTest, GoalSatisfiedFailsOnNullState)
{
  auto testable = std::make_shared<PassingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));
  EXPECT_FALSE(gr.isSatisfied(nullptr));

  auto state = si->allocState()->as<GeometricStateSpace::StateType>();
  stateSpace->freeState(state->mState);
  state->mState = nullptr;
  EXPECT_FALSE(gr.isSatisfied(state));
  si->freeState(state);
}

TEST_F(GoalRegionTest, GoalNotSatisfied)
{
  auto testable = std::make_shared<FailingConstraint>(stateSpace);
  auto generator
      = ::aikido::common::make_unique<EmptySampleGenerator>(stateSpace);
  GoalRegion gr(si, std::move(testable), std::move(generator));

  auto state = si->allocState();
  EXPECT_FALSE(gr.isSatisfied(state));
  si->freeState(state);
}
