#include <aikido/ompl/OMPLPlanner.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>
#include <ompl/base/SpaceInformation.h>
#include <boost/make_shared.hpp>

namespace aikido
{
    namespace ompl_bindings
    {
        template <class PlannerType>
        void planOMPL(
            const aikido::statespace::StateSpace::State *_start,
            const aikido::statespace::StateSpace::State *_goal,
            const std::shared_ptr<aikido::statespace::StateSpace> _stateSpace,
            const std::shared_ptr<aikido::constraint::TestableConstraint> _constraint,
            std::unique_ptr<util::RNG> _rng
            )
        {
            
            // Ensure the constraint and state space match
            if (_stateSpace != _constraint->getStateSpace()) {
                throw std::invalid_argument(
                    "StateSpace of constraint not equal to planning StateSpace"
                    );
            }

            // AIKIDO State space
            auto sspace = boost::make_shared<AIKIDOGeometricStateSpace>(_stateSpace, std::move(_rng));

            // Space information
            auto si = boost::make_shared<ompl::base::SpaceInformation>(sspace);
            
        }
    }
}
