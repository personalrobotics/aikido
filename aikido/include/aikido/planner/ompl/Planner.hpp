#ifndef AIKIDO_OMPL_OMPLPLANNER_HPP_
#define AIKIDO_OMPL_OMPLPLANNER_HPP_

#include "../../distance/DistanceMetric.hpp"
#include "../../statespace/StateSpace.hpp"
#include "../../statespace/Interpolator.hpp"
#include "../../constraint/Testable.hpp"
#include "../../constraint/Sampleable.hpp"
#include "../../constraint/Projectable.hpp"
#include "../../trajectory/Trajectory.hpp"

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <boost/make_shared.hpp>

namespace aikido {
namespace planner {
namespace ompl {

/// Use the template OMPL Planner type to plan a trajectory that moves from the
/// start to the goal point.
/// \param _start The start state
/// \param _goal The goal state
/// \param _statespace The StateSpace that the planner must plan within
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
template <class PlannerType>
trajectory::TrajectoryPtr planOMPL(
    const statespace::StateSpace::State *_start,
    const statespace::StateSpace::State *_goal,
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector, double _maxPlanTime);

/// Use the template OMPL Planner type to plan a trajectory that moves from the
/// start to a goal region.
/// \param _start The start state
/// \param _goalTestable A Testable constraint that can determine if a given state is a goal state
/// \param _goalSamplers A Sampleable capable of sampling states that satisfy _goalTestable
/// \param _statespace The StateSpace that the planner must plan within
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
template <class PlannerType>
trajectory::TrajectoryPtr planOMPL(
    const statespace::StateSpace::State *_start,
    constraint::TestablePtr _goalTestable,
    constraint::SampleablePtr _goalSampler,
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector,
    double _maxPlanTime);

/// Use the template OMPL Planner type to plan a trajectory that moves from the
/// start to a goal region while respecting a constraint
/// \param _start The start state
/// \param _goalTestable A Testable constraint that can determine if a given state is a goal state
/// \param _goalSamplers A Sampleable capable of sampling states that satisfy _goalTestable
/// \param _trajConstraint The constraint to satisfy along the trajectory
/// \param _statespace The StateSpace that the planner must plan within
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
template <class PlannerType>
trajectory::TrajectoryPtr planConstrained(
    const statespace::StateSpace::State *_start,
    constraint::TestablePtr _goalTestable,
    constraint::SampleablePtr _goalSampler,
    constraint::ProjectablePtr _trajConstraint,
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric, 
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector, 
    double _maxPlanTime);

/// Generate an OMPL SpaceInformation from aikido components
/// \param _statespace The StateSpace that the SpaceInformation operates on
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
::ompl::base::SpaceInformationPtr getSpaceInformation(
    statespace::StateSpacePtr _stateSpace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjector);

/// Use the template OMPL Planner type to plan in a custom OMPL Space
/// Information and problem definition and return an aikido Trajector
/// \param _si The SpaceInformation used by the planner
/// \param _pdef The ProblemDefintion. This contains start and goal conditions for the planner.
/// \param _sspace The aikido StateSpace to plan against. Used for constructing the return trajectory.
/// \param _interpolator An aikido interpolator that can be used with the _stateSpace.
/// \param _maxPlanTime The maximum time to allow the planner to search for a
/// solution
trajectory::TrajectoryPtr planOMPL(const ::ompl::base::PlannerPtr &_planner,
                             const ::ompl::base::ProblemDefinitionPtr &_pdef,
                             statespace::StateSpacePtr _sspace,
                             statespace::InterpolatorPtr _interpolator,
                             double _maxPlanTime);

} // namespace ompl
} // namespace planner
} // namespace aikido

#include "detail/Planner-impl.hpp"

#endif
