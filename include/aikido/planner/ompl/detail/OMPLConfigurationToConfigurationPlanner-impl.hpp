#ifndef AIKIDO_PLANNER_OMPL_DETAIL_OMPLCONFIGURATIONTOCONFIGURATION_IMPL_HPP_
#define AIKIDO_PLANNER_OMPL_DETAIL_OMPLCONFIGURATIONTOCONFIGURATION_IMPL_HPP_

#include "aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"

#include <utility>

#include "aikido/constraint/dart/FrameDifferentiable.hpp"
#include "aikido/constraint/dart/FrameTestable.hpp"
#include "aikido/constraint/dart/InverseKinematicsSampleable.hpp"
#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/distance/defaults.hpp"

#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/planner/ompl/CRRT.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/planner/ompl/MotionValidator.hpp>
#include <aikido/planner/ompl/Planner.hpp>

namespace aikido {
namespace planner {
namespace ompl {

using aikido::constraint::dart::createProjectableBounds;
using aikido::constraint::dart::createSampleableBounds;
using aikido::constraint::dart::createTestableBounds;
using aikido::distance::createDistanceMetric;

//==============================================================================
template <class PlannerType>
OMPLConfigurationToConfigurationPlanner<PlannerType>::OMPLConfigurationToConfigurationPlanner(
    statespace::ConstStateSpacePtr stateSpace,
    statespace::ConstInterpolatorPtr interpolator,
    distance::DistanceMetricPtr dmetric,
    constraint::SampleablePtr sampler,
    constraint::TestablePtr validityConstraint,
    constraint::TestablePtr boundsConstraint,
    constraint::ProjectablePtr boundsProjector,
    double maxDistanceBtwValidityChecks)
  : ConfigurationToConfigurationPlanner(std::move(stateSpace))
  , mInterpolator(std::move(interpolator))
{
    if (!mInterpolator)
    {
      mInterpolator
          = std::make_shared<aikido::statespace::GeodesicInterpolator>(mStateSpace);
    }

    // Geometric State space
    auto sspace = ompl_make_shared<GeometricStateSpace>(
        mStateSpace,
        std::make_shared<aikido::statespace::GeodesicInterpolator>(mStateSpace),
        std::move(dmetric),
        std::move(sampler),
        boundsConstraint,
        std::move(boundsProjector));

    // Space Information
    auto si = ompl_make_shared<::ompl::base::SpaceInformation>(std::move(sspace));

    // Validity checking
    std::vector<constraint::ConstTestablePtr> constraints{
        std::move(validityConstraint), std::move(boundsConstraint)};
    auto conjunctionConstraint
        = std::make_shared<constraint::TestableIntersection>(
            std::move(mStateSpace), std::move(constraints));
    ::ompl::base::StateValidityCheckerPtr vchecker
        = ompl_make_shared<StateValidityChecker>(si, conjunctionConstraint);
    si->setStateValidityChecker(vchecker);

    ::ompl::base::MotionValidatorPtr mvalidator
        = ompl_make_shared<MotionValidator>(si, maxDistanceBtwValidityChecks);
    si->setMotionValidator(mvalidator);

    // Setup the planner
    mPlanner = ompl_make_shared<PlannerType>(si);
}

//==============================================================================
template<class PlannerType>
trajectory::TrajectoryPtr OMPLConfigurationToConfigurationPlanner<PlannerType>::plan(
    const SolvableProblem& problem, Result* result)
{
    auto si = mPlanner->getSpaceInformation();

    // Define the OMPL problem
    auto pdef = ompl_make_shared<::ompl::base::ProblemDefinition>(si);
    auto sspace
        = ompl_static_pointer_cast<GeometricStateSpace>(si->getStateSpace());
    auto start = sspace->allocState(problem.getStartState());
    auto goal = sspace->allocState(problem.getGoalState());

    // ProblemDefinition clones states and keeps them internally
    pdef->setStartAndGoalStates(start, goal);

    sspace->freeState(start);
    sspace->freeState(goal);

    // Plan
    mPlanner->setProblemDefinition(pdef);
    mPlanner->setup();
    auto solved = mPlanner->solve(1);

    if (solved)
    {
      auto returnTraj =
          std::make_shared<trajectory::Interpolated>(mStateSpace, mInterpolator);

      // Get the path
      auto path = ompl_dynamic_pointer_cast<::ompl::geometric::PathGeometric>(
          pdef->getSolutionPath());
      if (!path)
      {
        throw std::invalid_argument(
            "Path is not of type PathGeometric. Cannot convert to aikido "
            "Trajectory.");
      }

      for (std::size_t idx = 0; idx < path->getStateCount(); ++idx)
      {
        const auto* st
            = static_cast<aikido::planner::ompl::GeometricStateSpace::StateType*>(path->getState(idx));
        returnTraj->addWaypoint(idx, st->mState);
      }

      return returnTraj;
    }

    if (result)
      result->setMessage("Problem could not be solved.");
    return nullptr;
}

//==============================================================================
template<class PlannerType>
::ompl::base::PlannerPtr OMPLConfigurationToConfigurationPlanner<PlannerType>::getOMPLPlanner()
{
  return mPlanner;
}

//==============================================================================
template<class PlannerType>
void OMPLConfigurationToConfigurationPlanner<PlannerType>::setInterpolator(
    statespace::ConstInterpolatorPtr interpolator)
{
  mInterpolator = std::move(interpolator);
}

//==============================================================================
template<class PlannerType>
statespace::ConstInterpolatorPtr
OMPLConfigurationToConfigurationPlanner<PlannerType>::getInterpolator() const
{
  return mInterpolator;
}

} // namespace ompl
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OMPL_DETAIL_OMPLCONFIGURATIONTOCONFIGURATION_IMPL_HPP_
