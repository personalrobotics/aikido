#ifndef AIKIDO_PLANNER_KINODYNAMICS_KINODYNAMICPLANNER_HPP_
#define AIKIDO_PLANNER_KINODYNAMICS_KINODYNAMICPLANNER_HPP_

#include <dart/dynamics/MetaSkeleton.hpp>
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/geometric/PathSimplifier.h>

#include "aikido/constraint/Projectable.hpp"
#include "aikido/constraint/Sampleable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/planner/kinodynamics/dimt/DoubleIntegratorMinimumTime.h"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Interpolated.hpp"
#include "aikido/trajectory/Spline.hpp"

namespace aikido {
namespace planner {
namespace kinodynamics {

/// Generate an OMPL SpaceInformation from aikido components
/// \param _dimt Minimum time double integrator
/// \param _validityConstraint A constraint used to test validity during
/// planning. This should include collision checking and any other constraints
/// that must be satisfied for a state to be considered valid.
/// \param _boundsProjector A Projectable that projects a state back within
/// valid bounds defined on the StateSpace
/// \param _maxDistanceBtwValidityChecks The maximum distance (under dmetric)
/// between validity checking two successive points on a tree extension
::ompl::base::SpaceInformationPtr getSpaceInformation(
    DIMTPtr _dimt,
    dart::dynamics::MetaSkeletonPtr _skeleton,
    constraint::TestablePtr _validityConstraint,
    double _maxDistanceBtwValidityChecks);

/// Use the template OMPL Planner type to plan a trajectory that moves from the
/// start to the goal point. Returns nullptr on planning failure.
/// \param _start The start state
/// \param _goal The goal state
/// \param _stateSpace The StateSpace that the planner must plan within
/// \param _interpolator An Interpolator defined on the StateSpace. This is used
/// to interpolate between two points within the space.
/// \param _dmetric A valid distance metric defined on the StateSpace
/// \param _sampler A Sampleable that can sample states from the
/// StateSpace. Warning: Many OMPL planners internally assume this sampler
/// samples uniformly. Care should be taken when using a non-uniform sampler.
/// \param _validityConstraint A constraint used to test validity during
/// planning. This should include collision checking and any other constraints
/// that must be satisfied for a state to be considered valid.
/// \param _boundsConstraint A constraint used to determine whether states
/// encountered during planning fall within any bounds specified on the
/// StateSpace. In addition to the _validityConstraint, this must also be
/// satsified for a state to be considered valid.
/// \param _maxPlanTime The maximum time to allow the planner to search for a
/// solution
/// \param _maxDistanceBtwValidityChecks The maximum distance (under dmetric)
/// between validity checking two successive points on a tree extension
std::unique_ptr<aikido::trajectory::Spline> planMinimumTimeViaConstraint(
    const statespace::StateSpace::State* _start,
    const statespace::StateSpace::State* _goal,
    const statespace::StateSpace::State* _via,
    const Eigen::VectorXd& _viaVelocity,
    dart::dynamics::MetaSkeletonPtr _metaSkeleton,
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    constraint::TestablePtr _validityConstraint,
    double& _viaTime,
    double _maxPlanTime,
    double _maxDistanceBtwValidityChecks);

} // namespace kinodynamics
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_KINODYNAMICS_KINODYNAMICPLANNER_HPP_
