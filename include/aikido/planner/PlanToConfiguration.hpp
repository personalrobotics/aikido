#ifndef AIKIDO_PLANNER_PLANTOCONFIGURATION_HPP_
#define AIKIDO_PLANNER_PLANTOCONFIGURATION_HPP_

#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/smart_pointer.hpp"
#include "aikido/constraint/smart_pointer.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a single goal configuration.
class PlanToConfiguration : public Problem
{
public:
  using ReturnTrajectoryType = trajectory::Interpolated;

  class Result;

  PlanToConfiguration(
      statespace::ConstStateSpacePtr stateSpace,
      const statespace::StateSpace::State* startState,
      const statespace::StateSpace::State* goalState,
      statespace::InterpolatorPtr interpolator,
      constraint::TestablePtr constraint);

  const std::string& getName() const override;

  static const std::string& getStaticName();

  const statespace::StateSpace::State* getStartState() const;

  const statespace::StateSpace::State* getGoalState() const;

  statespace::InterpolatorPtr getInterpolator();

  statespace::ConstInterpolatorPtr getInterpolator() const;

  constraint::TestablePtr getConstraint();

  constraint::ConstTestablePtr getConstraint() const;

protected:
  const statespace::StateSpace::State* mStartState;
  const statespace::StateSpace::State* mGoalState;

  statespace::InterpolatorPtr mInterpolator;
  constraint::TestablePtr mConstraint;
};

class PlanToConfiguration::Result : public Problem::Result
{
public:
protected:
};

} // namespace planner
} // namespace aikido

#endif
