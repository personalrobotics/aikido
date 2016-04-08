#ifndef DART_GEOMETRIC_STATE_SPACE_H
#define DART_GEOMETRIC_STATE_SPACE_H

#include <ompl/base/StateValidityChecker.h>
#include <aikido/statespace/StateSpace.hpp>

namespace aikido {
    namespace ompl_bindings {

        class DARTGeometricStateValidityChecker : public ompl::base::StateValidityChecker {

        public:
            
            DARTGeometricStateValidityChecker(const ompl::base::SpaceInformationPtr &si);
            
            /// Return true if the state state is valid. Usually, this means at 
            /// least collision checking and bounds checking
            virtual bool isValid(const ompl::base::State* _state) const;

        };
    }
}
#endif
