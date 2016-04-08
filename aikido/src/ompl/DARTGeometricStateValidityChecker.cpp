#include <aikido/ompl/DARTGeometricStateValidityChecker.hpp>

namespace aikido {
    namespace ompl_bindings {

        DARTGeometricStateValidityChecker::DARTGeometricStateValidityChecker(const ompl::base::SpaceInformationPtr &si)
            : ompl::base::StateValidityChecker(si) {

        }

        bool DARTGeometricStateValidityChecker::isValid(const ompl::base::State* _state) const {

            auto stateSpace = si_->getStateSpace();

            // Check bounds
            if(!stateSpace->satisfiesBounds(_state)){
                return false;
            }

            // Check collision
            
        }
    }
}
