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

/// Planning problem to plan to given single Task Space Region (TSR).
class PlanToTSR : public Problem
{
public:
  class Result;

  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param bodyNode Body Node or robot for which the path is to be planned.
  /// \param startState Start state.
  /// \param goalTSR Goal TSR.
  /// \param interpolator Interpolator used to produce the output trajectory.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible to \c constraint's state space.
  PlanToTSR(
      statespace::StateSpacePtr stateSpace,
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
  /// Body Node or Robot
  dart::dynamics::BodyNodePtr mBodyNode;

  /// Start State
  const statespace::StateSpace::State* mStartState;

  /// Goal TSR
  const constraint::TSRPtr mGoalTSR;

  /// Interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr mInterpolator;

  /// Trajectory-wide constraint that must be satisfied.
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
