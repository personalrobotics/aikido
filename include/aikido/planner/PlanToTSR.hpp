#ifndef AIKIDO_PLANNER_PLANTOTSR_HPP_
#define AIKIDO_PLANNER_PLANTOTSR_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/TSR.hpp"
#include "aikido/constraint/smart_pointer.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/smart_pointer.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to given Task Space Region (TSR).
class PlanToTSR : public Problem
{
public:
  using ReturnTrajectoryType = trajectory::Interpolated;

  class Result;

  PlanToTSR(
      statespace::ConstStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bodyNode,
      const statespace::StateSpace::State* startState,
      const constraint::TSRPtr goalTSR,
      statespace::InterpolatorPtr interpolator,
      constraint::TestablePtr constraint);

  const std::string& getName() const override;

  static const std::string& getStaticName();

  dart::dynamics::BodyNodePtr getBodyNode();

  const statespace::StateSpace::State* getStartState() const;

  const constraint::TSRPtr getGoalTSR() const;

  statespace::InterpolatorPtr getInterpolator();

  statespace::ConstInterpolatorPtr getInterpolator() const;

  constraint::TestablePtr getConstraint();

  constraint::ConstTestablePtr getConstraint() const;

protected:
  dart::dynamics::BodyNodePtr mBodyNode;

  const statespace::StateSpace::State* mStartState;
  const constraint::TSRPtr mGoalTSR;

  statespace::InterpolatorPtr mInterpolator;
  constraint::TestablePtr mConstraint;
};

class PlanToTSR::Result : public Problem::Result
{
public:
protected:
};

} // namespace planner
} // namespace aikido

#endif
