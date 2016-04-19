#ifndef AIKIDO_OMPL_PLANNER_H_
#define AIKIDO_OMPL_PLANNER_H_

#include "../distance/DistanceMetric.hpp"
#include "../statespace/StateSpace.hpp"
#include "../statespace/Interpolator.hpp"
#include "../constraint/TestableConstraint.hpp"
#include "../constraint/Sampleable.hpp"
#include "../constraint/Projectable.hpp"
#include "../path/Trajectory.hpp"

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <boost/make_shared.hpp>

namespace aikido
{
namespace ompl
{
::ompl::base::SpaceInformationPtr getSpaceInformation(
    const statespace::StateSpacePtr &_stateSpace,
    const statespace::InterpolatorPtr &_interpolator,
    const distance::DistanceMetricPtr &_dmetric,
    const constraint::SampleableConstraintPtr &_sampler,
    const constraint::TestableConstraintPtr &_boundsConstraint,
    const constraint::ProjectablePtr &_boundsProjector);

void setValidityConstraints(
    const ::ompl::base::SpaceInformationPtr &_si,
    const constraint::TestableConstraintPtr &_collConstraint,
    const constraint::TestableConstraintPtr &_boundsConstraint);

}
}

#include "detail/OMPLPlanner.hpp"

#endif
