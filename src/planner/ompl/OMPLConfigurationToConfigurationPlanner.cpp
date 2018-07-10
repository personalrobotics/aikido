//#include "aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp"

//#include "aikido/constraint/Testable.hpp"
//#include "aikido/statespace/StateSpace.hpp"

//namespace aikido {
//namespace planner {
//namespace ompl {

////==============================================================================
//template<class PlannerType>
//trajectory::TrajectoryPtr OMPLConfigurationToConfigurationPlanner::plan(
//    const SolvableProblem& problem, Result* result)
//{
//    auto si = mPlanner->getSpaceInformation();

//    // Define the OMPL problem
//    auto pdef = ompl_make_shared<::ompl::base::ProblemDefinition>(si);
//    auto sspace
//        = ompl_static_pointer_cast<GeometricStateSpace>(si->getStateSpace());
//    auto start = sspace->allocState(problem.getStartState());
//    auto goal = sspace->allocState(problem.getGoalState());

//    // ProblemDefinition clones states and keeps them internally
//    pdef->setStartAndGoalStates(start, goal);

//    sspace->freeState(start);
//    sspace->freeState(goal);

//    // Plan
//    mPlanner->setProblemDefinition(pdef);
//    mPlanner->setup();
//    auto solved = mPlanner->solve(maxPlanTime);

//    if (solved)
//    {
//      auto returnTraj =
//          std::make_shared<trajectory::Interpolated>(mStateSpace, mInterpolator);

//      // Get the path
//      auto path = ompl_dynamic_pointer_cast<::ompl::geometric::PathGeometric>(
//          pdef->getSolutionPath());
//      if (!path)
//      {
//        throw std::invalid_argument(
//            "Path is not of type PathGeometric. Cannot convert to aikido "
//            "Trajectory.");
//      }

//      for (std::size_t idx = 0; idx < path->getStateCount(); ++idx)
//      {
//        const auto* st
//            = static_cast<aikido::planner::ompl::GeometricStateSpace::StateType*>(path->getState(idx));
//        returnTraj->addWaypoint(idx, st->mState);
//      }

//      return returnTraj;
//    }

//    if (result)
//      result->setMessage("Problem could not be solved.");
//    return nullptr;
//}

////==============================================================================
//template<class PlannerType>
//::ompl::base::PlannerPtr OMPLConfigurationToConfigurationPlanner::getOMPLPlanner()
//{
//  return mPlanner;
//}

////==============================================================================
//template<class PlannerType>
//void OMPLConfigurationToConfigurationPlanner::setInterpolator(
//    statespace::ConstInterpolatorPtr interpolator)
//{
//  mInterpolator = std::move(interpolator);
//}

////==============================================================================
//template<class PlannerType>
//statespace::ConstInterpolatorPtr
//OMPLConfigurationToConfigurationPlanner::getInterpolator() const
//{
//  return mInterpolator;
//}

//} // namespace ompl
//} // namespace planner
//} // namespace aikido
