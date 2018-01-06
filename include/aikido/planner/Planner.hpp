#ifndef AIKIDO_PLANNER_PLANNER_HPP_
#define AIKIDO_PLANNER_PLANNER_HPP_

namespace aikido {
namespace planner {

/// Base class for a meta-planner, to be implemented.
/// [Gilwoo] Created a strawman for Robot class
class Planner
{

  aikido::trajectory::TrajectoryPtr planToConfiguration(
      const statespace::StateSpacePtr& stateSpace,
      const statespace::StateSpace::State* startState,
      const statespace::StateSpace::State* goalState,
      const CollisionFreePtr& collisionFree,
      double timeLimit);

  aikido::trajectory::TrajectoryPtr planToConfigurations(
      const statespace::StateSpacePtr& stateSpace,
      const statespace::StateSpace::State* startState,
      const std::vector<statespace::StateSpace::State*> goalStates,
      const CollisionFreePtr& collisionFree,
      double timeLimit);

  aikido::trajectory::TrajectoryPtr planToTSR(
      const statespace::StateSpacePtr& stateSpace,
      const statespace::StateSpace::State* startState,
      const aikido::constraint::TSR& tsr,
      const CollisionFreePtr& collisionFree,
      double timelimit);
}

using PlannerPtr = std::shared_ptr<Planner>;

} // namespace planner
} // namespace aikido
