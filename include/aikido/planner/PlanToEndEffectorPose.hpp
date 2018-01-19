#ifndef AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_
#define AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_

#include <dart/dart.hpp>
#include "aikido/constraint/smart_pointer.hpp"
#include "aikido/planner/Problem.hpp"
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/smart_pointer.hpp"
#include "aikido/trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {

/// Planning problem to plan to a multiple goal configurations.
class PlanToEndEffectorPose : public Problem
{
public:
  class Result;

  PlanToEndEffectorPose(
      statespace::ConstStateSpacePtr stateSpace,
      dart::dynamics::BodyNodePtr bodyNode,
      const statespace::StateSpace::State* startState,
      const statespace::StateSpace::State* goalState,
      const Eigen::Vector3d& direction,
      statespace::InterpolatorPtr interpolator,
      constraint::TestablePtr constraint,
      const Eigen::Isometry3d& goalPose);

  const std::string& getName() const override;

  static const std::string& getStaticName();

  dart::dynamics::BodyNodePtr getBodyNode();

  const statespace::StateSpace::State* getStartState() const;

  const statespace::StateSpace::State* getGoalState() const;

  const Eigen::Vector3d& getDirection() const;

  statespace::InterpolatorPtr getInterpolator();

  statespace::ConstInterpolatorPtr getInterpolator() const;

  constraint::TestablePtr getConstraint();

  constraint::ConstTestablePtr getConstraint() const;

  const Eigen::Isometry3d& getGoalPose() const;

protected:
  dart::dynamics::BodyNodePtr mBodyNode;

  const statespace::StateSpace::State* mStartState;
  const statespace::StateSpace::State* mGoalState;

  const Eigen::Vector3d mDirection;

  statespace::InterpolatorPtr mInterpolator;
  constraint::TestablePtr mConstraint;
  Eigen::Isometry3d mGoalPose;
};

class PlanToEndEffectorPose::Result : public Problem::Result
{
public:
protected:
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANTOENDEFFECTORPOSE_HPP_
