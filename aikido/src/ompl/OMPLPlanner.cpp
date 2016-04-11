#include <aikido/ompl/OMPLPlanner.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>
#include <aikido/ompl/AIKIDOStateValidityChecker.hpp>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>

namespace aikido
{
    namespace ompl_bindings
    {
        template <class PlannerType>
        void planOMPL(
            const aikido::statespace::StateSpace::State *_start,
            const aikido::statespace::StateSpace::State *_goal,
            const std::shared_ptr<aikido::statespace::StateSpace> &_stateSpace,
            const std::shared_ptr<aikido::constraint::TestableConstraint> &_constraint,
            const double &_maxPlanTime,
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
            auto sspace = std::make_shared<AIKIDOGeometricStateSpace>(_stateSpace, std::move(_rng));

            // Space Information
            auto si = std::make_shared<ompl::base::SpaceInformation>(sspace);

            // Validity checker
            std::vector<std::shared_ptr<aikido::constraint::TestableConstraint> > constraints;
            constraints.push_back(_constraint);
            ompl::base::StateValidityCheckerPtr vchecker = 
                std::make_shared<AIKIDOStateValidityChecker>(si, constraints);
            si->setStateValidityChecker(vchecker);

            // Start and states
            auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
            auto start = sspace->allocState(_start);
            auto goal = sspace->allocState(_goal);
            pdef->setStartAndGoalStates(start, goal);
            
            // Planner
            auto planner = std::make_shared<PlannerType>(si);
            planner->setProblemDefinition(pdef);
            planner->setup();
            auto solved = planner->solve(_maxPlanTime);
            
            if(solved){
                // TODO: Extract trajectory
            }
        }
    }
}
