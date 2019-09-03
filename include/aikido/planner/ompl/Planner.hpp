#ifndef AIKIDO_OMPL_OMPLPLANNER_HPP_
#define AIKIDO_OMPL_OMPLPLANNER_HPP_

#include <chrono>
#include <utility> // std::pair

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/geometric/PathSimplifier.h>

#include "../../constraint/Projectable.hpp"
#include "../../constraint/Sampleable.hpp"
#include "../../constraint/Testable.hpp"
#include "../../distance/DistanceMetric.hpp"
#include "../../planner/ompl/BackwardCompatibility.hpp"
#include "../../planner/ompl/GeometricStateSpace.hpp"
#include "../../statespace/Interpolator.hpp"
#include "../../statespace/StateSpace.hpp"
#include "../../trajectory/Interpolated.hpp"

namespace aikido {
namespace planner {
namespace ompl {

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
template <class PlannerType>
trajectory::InterpolatedPtr planOMPL(
    const statespace::StateSpace::State* _start,
    const statespace::StateSpace::State* _goal,
    statespace::ConstStateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxPlanTime,
    double _maxDistanceBtwValidityChecks);

/// Use the template OMPL Planner type to plan a trajectory that moves from the
/// start to a goal region. Returns nullptr on planning failure.
/// \param _start The start state
/// \param _goalTestable A Testable constraint that can determine if a given
/// state is a goal state
/// \param _goalSampler A Sampleable capable of sampling states that satisfy
/// _goalTestable
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
template <class PlannerType>
trajectory::InterpolatedPtr planOMPL(
    const statespace::StateSpace::State* _start,
    constraint::TestablePtr _goalTestable,
    constraint::SampleablePtr _goalSampler,
    statespace::ConstStateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxPlanTime,
    double _maxDistanceBtwValidityChecks);

/// Use the CRRT planner to plan a trajectory that moves from the
/// start to a goal region while respecting a constraint
/// \param _start The start state
/// \param _goalTestable A Testable constraint that can determine if a given
/// state is a goal state
/// \param _goalSampler A Sampleable capable of sampling states that satisfy
/// _goalTestable
/// \param _trajConstraint The constraint to satisfy along the trajectory
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
/// \param _maxExtensionDistance The maximum distance to extend the tree on
///  a single extension
/// \param _maxDistanceBtwProjections The maximum distance (under dmetric)
/// between projecting and validity checking two successive points on a tree
/// extension
/// \param _minStepsize The minimum distance between two states for the them to
/// be considered "different"
trajectory::InterpolatedPtr planCRRT(
    const statespace::StateSpace::State* _start,
    constraint::TestablePtr _goalTestable,
    constraint::SampleablePtr _goalSampler,
    constraint::ProjectablePtr _trajConstraint,
    statespace::ConstStateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxPlanTime,
    double _maxExtensionDistance,
    double _maxDistanceBtwProjections,
    double _minStepsize);

/// Use the CRRT planner to plan a trajectory that moves from the
/// start to a goal region while respecting a constraint
/// \param _start The start state
/// \param _goalTestable A Testable constraint that can determine if a given
/// state is a goal state
/// \param _goalSampler A Sampleable capable of sampling states that satisfy
/// _goalTestable
/// \param _trajConstraint The constraint to satisfy along the trajectory
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
/// \param _maxExtensionDistance The maximum distance to extend the tree on
///  a single extension
/// \param _maxDistanceBtwProjections The maximum distance (under dmetric)
/// between projecting and validity checking two successive points on a tree
/// extension
/// \param _minTreeConnectionDistance The minumum distance between the start and
/// goal tree to consider them connected
/// \param _minStepsize The minimum distance between two states for the them to
/// be considered "different"
trajectory::InterpolatedPtr planCRRTConnect(
    const statespace::StateSpace::State* _start,
    constraint::TestablePtr _goalTestable,
    constraint::SampleablePtr _goalSampler,
    constraint::ProjectablePtr _trajConstraint,
    statespace::ConstStateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxPlanTime,
    double _maxExtensionDistance,
    double _maxDistanceBtwProjections,
    double _minStepsize,
    double _minTreeConnectionDistance);

/// Generate an OMPL SpaceInformation from aikido components
/// \param _stateSpace The StateSpace that the SpaceInformation operates on
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
/// \param _maxDistanceBtwValidityChecks The maximum distance (under dmetric)
/// between validity checking two successive points on a tree extension
::ompl::base::SpaceInformationPtr getSpaceInformation(
    statespace::ConstStateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxDistanceBtwValidityChecks);

/// Create an OMPL GoalRegion from a Testable and Sampler that describe the goal
/// region
/// \param _si Information about the planning space
/// \param _goalTestable A Testable constraint that can determine if a given
/// state is a goal state
/// \param _goalSampler A Sampleable capable of sampling states that satisfy
/// _goalTestable
ompl_shared_ptr<::ompl::base::GoalRegion> getGoalRegion(
    ::ompl::base::SpaceInformationPtr _si,
    constraint::TestablePtr _goalTestable,
    constraint::SampleablePtr _goalSampler);

/// Use the template OMPL Planner type to plan in a custom OMPL Space
/// Information and problem definition and return an aikido Trajector
/// Returns nullptr on planning failure.
/// \param _planner Points to some OMPL planner.
/// \param _pdef The ProblemDefintion. This contains start and goal conditions
/// for the planner.
/// \param _sspace The aikido StateSpace to plan against. Used for constructing
/// the return trajectory.
/// \param _interpolator An aikido interpolator that can be used with the
/// _stateSpace.
/// \param _maxPlanTime The maximum time to allow the planner to search for a
/// solution
trajectory::InterpolatedPtr planOMPL(
    const ::ompl::base::PlannerPtr& _planner,
    const ::ompl::base::ProblemDefinitionPtr& _pdef,
    statespace::ConstStateSpacePtr _sspace,
    statespace::InterpolatorPtr _interpolator,
    double _maxPlanTime);

/// Take in an aikido trajectory and simplify it using OMPL methods
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
/// \param _maxDistanceBtwValidityChecks The maximum distance (under dmetric)
/// between validity checking two successive points on a tree extension
/// \param _timeout Timeout, in seconds, after which the simplifier terminates
/// to return possibly shortened path
/// \param _maxEmptySteps Maximum number of consecutive failed attempts at
/// shortening before simplification terminates. Default 0, equal to number
/// of states in the path
/// \param _originalTraj The untimed trajectory obtained from the planner,
/// needs simplifying.
std::pair<std::unique_ptr<trajectory::Interpolated>, bool> simplifyOMPL(
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxDistanceBtwValidityChecks,
    double _timeout,
    std::size_t _maxEmptySteps,
    trajectory::InterpolatedPtr _originalTraj);

/// Take an interpolated trajectory and convert it into OMPL geometric path
/// \param _interpolatedTraj the interpolated trajectory to be converted
/// \param _si Information about the planning space
/// returns the corresponding OMPL geometric path
::ompl::geometric::PathGeometric toOMPLTrajectory(
    const trajectory::InterpolatedPtr& _interpolatedTraj,
    ::ompl::base::SpaceInformationPtr _si);

/// Take an OMPL geometric path and convert it into an interpolated trajectory
/// \param _path The OMPL geometric path
/// \param _interpolator An Interpolator defined on the StateSpace. This is used
/// to interpolate between two points within the space.
/// returns the corresponding interpolated trajectory
std::unique_ptr<trajectory::Interpolated> toInterpolatedTrajectory(
    const ::ompl::geometric::PathGeometric& _path,
    statespace::InterpolatorPtr _interpolator);

} // namespace ompl
} // namespace planner
} // namespace aikido

#include "detail/Planner-impl.hpp"

#endif
