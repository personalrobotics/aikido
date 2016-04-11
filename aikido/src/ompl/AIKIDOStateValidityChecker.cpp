#include <aikido/ompl/AIKIDOStateValidityChecker.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>

namespace aikido {
    namespace ompl_bindings {

        AIKIDOStateValidityChecker::AIKIDOStateValidityChecker(const ompl::base::SpaceInformationPtr &_si,
                                                           std::vector<std::shared_ptr<
                                                           aikido::constraint::TestableConstraint> > _constraints)
            : ompl::base::StateValidityChecker(_si), mConstraints(_constraints)
        {

        }

        bool AIKIDOStateValidityChecker::isValid(const ompl::base::State* _state) const {

            auto stateSpace = si_->getStateSpace();

            // Check bounds
            if(!stateSpace->satisfiesBounds(_state)){
                return false;
            }

            // Check constraints
            auto state = static_cast<const AIKIDOGeometricStateSpace::StateType*>(_state);
            for (auto& constraint : mConstraints)
            {
                if(!constraint->isSatisfied(state->mState)){
                    return false;
                }
            }
            
            return true;
        }
    }
}
