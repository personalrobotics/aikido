#ifndef AIKIDO_OMPL_PLANNER_H_
#define AIKIDO_OMPL_PLANNER_H_

#include "../distance/DistanceMetric.hpp"
#include "../statespace/StateSpace.hpp"
#include "../constraint/TestableConstraint.hpp"
#include "../constraint/Sampleable.hpp"

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <boost/make_shared.hpp>

namespace aikido {
namespace ompl {

template <class PlannerType>
void planOMPL(
    const aikido::statespace::StateSpace::State *_start,
    const aikido::statespace::StateSpace::State *_goal,
    const std::shared_ptr<aikido::statespace::StateSpace> &_stateSpace,
    const std::shared_ptr<aikido::constraint::TestableConstraint> &_constraint,
    const aikido::distance::DistanceMetricPtr &_dmetric,
    std::unique_ptr<aikido::constraint::SampleableConstraint> _sampler,
    const double &_maxPlanTime, std::unique_ptr<util::RNG> _rng);
}
}

#include "detail/OMPLPlanner.hpp"

#endif
