#ifndef AIKIDO_OMPL_OMPLPLANNER_HPP_
#define AIKIDO_OMPL_OMPLPLANNER_HPP_

#include "../../constraint/Projectable.hpp"
#include "../../constraint/Sampleable.hpp"
#include "../../constraint/Testable.hpp"
#include "../../distance/DistanceMetric.hpp"
#include "../../planner/ompl/BackwardCompatibility.hpp"
#include "../../planner/ompl/GeometricStateSpace.hpp"
#include "../../statespace/Interpolator.hpp"
#include "../../statespace/StateSpace.hpp"
#include "../../trajectory/Interpolated.hpp"

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalRegion.h>

#include <ompl/geometric/PathSimplifier.h>

namespace aikido {
namespace planner {
namespace kinodynamics {

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
/// \param _boundsProjector A Projectable that projects a state back within
/// valid bounds defined on the StateSpace
/// \param _maxPlanTime The maximum time to allow the planner to search for a
/// solution
/// \param _maxDistanceBtwValidityChecks The maximum distance (under dmetric)
/// between validity checking two successive points on a tree extension
trajectory::InterpolatedPtr planViaConstraint(
    const statespace::StateSpace::State* _start,
    const statespace::StateSpace::State* _goal,
    const statespace::StateSpace::State* _via,
    const Eigen::VectorXd& _viaVelocity,
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxPlanTime,
    double _maxDistanceBtwValidityChecks);

} // namespace kinodynamics
} // namespace planner
} // namespace aikido

#endif
