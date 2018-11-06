#ifndef AIKIDO_PLANNER_OMPL_DETAIL_OMPLCONFIGURATIONTOCONFIGURATION_IMPL_HPP_
#define AIKIDO_PLANNER_OMPL_DETAIL_OMPLCONFIGURATIONTOCONFIGURATION_IMPL_HPP_

#include <utility>
#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/constraint/dart/FrameDifferentiable.hpp"
#include "aikido/constraint/dart/FrameTestable.hpp"
#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/planner/ompl/GeometricStateSpace.hpp"
#include "aikido/planner/ompl/MotionValidator.hpp"
#include "aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/ompl/Planner.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"

namespace aikido {
namespace planner {
namespace ompl {

using aikido::constraint::dart::createProjectableBounds;
using aikido::constraint::dart::createSampleableBounds;
using aikido::constraint::dart::createTestableBounds;
using aikido::constraint::dart::createProjectableBounds;
using aikido::distance::createDistanceMetric;
using aikido::statespace::dart::MetaSkeletonStateSpace;

//==============================================================================
template <class PlannerType>
OMPLConfigurationToConfigurationPlanner<PlannerType>::
    OMPLConfigurationToConfigurationPlanner(
        statespace::ConstStateSpacePtr stateSpace,
        common::RNG* rng,
        statespace::ConstInterpolatorPtr interpolator,
        distance::DistanceMetricPtr dmetric,
        constraint::SampleablePtr sampler,
        constraint::TestablePtr boundsConstraint,
        constraint::ProjectablePtr boundsProjector,
        double maxDistanceBetweenValidityChecks)
  : ConfigurationToConfigurationPlanner(std::move(stateSpace), rng)
{
  if (!interpolator)
    interpolator = std::make_shared<const aikido::statespace::GeodesicInterpolator>(
        mStateSpace);

  if (!dmetric)
    dmetric = std::move(createDistanceMetric(mStateSpace));

  // TODO (avk): The constraint namespace functions only work with
  // MetaSkeletonStateSpaces
  const auto metaskeletonStatespace
      = std::dynamic_pointer_cast<const MetaSkeletonStateSpace>(mStateSpace);

  if (!metaskeletonStatespace
      && (!sampler || !boundsConstraint || !boundsProjector))
  {
    throw std::invalid_argument(
        "[OMPLPlanner] Either statespace must be metaSkeletonStateSpace or all "
        "of sampler, boundsContraint and boundsProjector must be provided.");
  }

  if (!sampler)
  {
    if (!rng)
      throw std::invalid_argument(
          "[OMPLPlanner] Both of sampler and rng cannot be nullptr.");
    sampler = std::move(
        createSampleableBounds(metaskeletonStatespace, rng->clone()));
  }

  if (!boundsConstraint)
    boundsConstraint = std::move(createTestableBounds(metaskeletonStatespace));

  if (!boundsProjector)
    boundsProjector
        = std::move(createProjectableBounds(metaskeletonStatespace));

  // Geometric State space
  // TODO(avk): Also add in the maxDistanceCheck
  auto sspace = ompl_make_shared<GeometricStateSpace>(
      mStateSpace,
      std::move(interpolator),
      std::move(dmetric),
      std::move(sampler),
      std::move(boundsConstraint),
      std::move(boundsProjector),
      maxDistanceBetweenValidityChecks);

  // Space Information
  auto si = ompl_make_shared<::ompl::base::SpaceInformation>(sspace);

  // Setup the planner
  mPlanner = ompl_make_shared<PlannerType>(si);
}

//==============================================================================
template <class PlannerType>
trajectory::TrajectoryPtr
OMPLConfigurationToConfigurationPlanner<PlannerType>::plan(
    const SolvableProblem& problem, Result* result)
{
  auto si = mPlanner->getSpaceInformation();
  
  // Only geometric statespaces are supported.
  assert(ompl_dynamic_pointer_cast<GeometricStateSpace>(si->getStateSpace()));
  auto sspace = ompl_static_pointer_cast<GeometricStateSpace>(si->getStateSpace());

  // Set validity checker
  std::vector<constraint::ConstTestablePtr> constraints{problem.getConstraint(),
                                                        sspace->getBoundsConstraint()
                                                        };
  auto conjunctionConstraint
      = std::make_shared<constraint::TestableIntersection>(
          mStateSpace, std::move(constraints));
  ::ompl::base::StateValidityCheckerPtr vchecker
      = ompl_make_shared<StateValidityChecker>(si, conjunctionConstraint);
  si->setStateValidityChecker(vchecker);

  ::ompl::base::MotionValidatorPtr mvalidator
      = ompl_make_shared<MotionValidator>(si, sspace->getMaxDistanceBetweenValidityChecks());
  si->setMotionValidator(mvalidator);

  // Define the OMPL problem
  auto pdef = ompl_make_shared<::ompl::base::ProblemDefinition>(si);

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
    auto returnTraj = std::make_shared<trajectory::Interpolated>(
        mStateSpace, sspace->getInterpolator());

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
          = static_cast<aikido::planner::ompl::GeometricStateSpace::StateType*>(
              path->getState(idx));
      returnTraj->addWaypoint(idx, st->mState);
    }
    // Clear the planner internal data.
    mPlanner->clear();
    return returnTraj;
  }

  if (result)
    result->setMessage("Problem could not be solved.");
  mPlanner->clear();
  return nullptr;
}

//==============================================================================
template <class PlannerType>
::ompl::base::PlannerPtr
OMPLConfigurationToConfigurationPlanner<PlannerType>::getOMPLPlanner()
{
  return mPlanner;
}

} // namespace ompl
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OMPL_DETAIL_OMPLCONFIGURATIONTOCONFIGURATION_IMPL_HPP_
