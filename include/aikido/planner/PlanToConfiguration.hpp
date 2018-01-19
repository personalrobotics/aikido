#ifndef AIKIDO_PLANNER_PLANTOCONFIGURATION_HPP_
#define AIKIDO_PLANNER_PLANTOCONFIGURATION_HPP_

#include "aikido/constraint/smart_pointer.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/smart_pointer.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a single goal configuration.
///
/// Plan a trajectory from start state to goal state by using an interpolator to
/// interpolate between them.
class PlanToConfiguration : public Problem
{
public:
  using ReturnTrajectoryType = trajectory::Interpolated;

  class Result;

  /// Constructor.
  ///
  /// \param stateSpace State space.
  /// \param startState Start state.
  /// \param goalState Goal state.
  /// \param interpolator Interpolator used to produce the output trajectory.
  /// \param constraint Trajectory-wide constraint that must be satisfied.
  /// \throw If \c stateSpace is not compatible to \c constraint's state space.
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
  /// Start state.
  const statespace::StateSpace::State* mStartState;

  /// Goal state.
  const statespace::StateSpace::State* mGoalState;

  /// Interpolator used to produce the output trajectory.
  statespace::InterpolatorPtr mInterpolator;

  /// Trajectory-wide constraint that must be satisfied.
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
