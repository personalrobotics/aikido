#ifndef AIKIDO_PLANNER_PLANTOCONFIGURATIONS_HPP_
#define AIKIDO_PLANNER_PLANTOCONFIGURATIONS_HPP_

#include "aikido/constraint/smart_pointer.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/smart_pointer.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a multiple goal configurations.
class PlanToConfigurations : public Problem
{
public:
  class Result;

  PlanToConfigurations(
      statespace::ConstStateSpacePtr stateSpace,
      const statespace::StateSpace::State* startState,
      const std::vector<statespace::StateSpace::State*> goalStates,
      statespace::InterpolatorPtr interpolator,
      constraint::TestablePtr constraint);

  const std::string& getName() const override;

  static const std::string& getStaticName();

  const statespace::StateSpace::State* getStartState() const;

  const std::vector<statespace::StateSpace::State*> getGoalStates() const;

  statespace::InterpolatorPtr getInterpolator();

  statespace::ConstInterpolatorPtr getInterpolator() const;

  constraint::TestablePtr getConstraint();

  constraint::ConstTestablePtr getConstraint() const;

protected:
  const statespace::StateSpace::State* mStartState;
  const std::vector<statespace::StateSpace::State*> mGoalStates;

  statespace::InterpolatorPtr mInterpolator;
  constraint::TestablePtr mConstraint;
};

class PlanToConfigurations::Result : public Problem::Result
{
public:
protected:
};

} // namespace planner
} // namespace aikido

#endif
