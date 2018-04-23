#include <aikido/planner/kinodynamics/KinodynamicPlanner.hpp>
#include <aikido/planner/ompl/Planner.hpp>

namespace aikido {
namespace planner {
namespace kinodynamics {

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
    double _maxDistanceBtwValidityChecks)
{
    // plan from start to via

    // plan from via to goal

    // concatenate two path

}

} // namespace kinodynamics
} // namespace planner
} // namespace aikido
