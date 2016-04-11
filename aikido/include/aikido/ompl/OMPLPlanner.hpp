#ifndef AIKIDO_OMPL_PLANNER_H_
#define AIKIDO_OMPL_PLANNER_H_

#include <aikido/statespace/StateSpace.hpp>
#include <aikido/constraint/TestableConstraint.hpp>

namespace aikido
{
namespace ompl_bindings
{

    template <class PlannerType>
    static void planOMPL(
        const aikido::statespace::StateSpace::State *_start,
        const aikido::statespace::StateSpace::State *_goal,
        const std::shared_ptr<aikido::statespace::StateSpace> &_stateSpace,
        const std::shared_ptr<aikido::constraint::TestableConstraint> &_constraint,
        const double &_maxPlanTime,
        std::unique_ptr<util::RNG> _rng
        );

}
}

#endif
