#ifndef R3_OMPL_DARTGEOMETRICSTATEVALIDITYCHECKER_H_
#define R3_OMPL_DARTGEOMETRICSTATEVALIDITYCHECKER_H_
#include <ompl/base/StateValidityChecker.h>

namespace r3 {
namespace ompl {

class DARTGeometricStateSpace;

class DARTGeometricStateValidityChecker
        : public ::ompl::base::StateValidityChecker {
public:
    DARTGeometricStateValidityChecker(
        ::ompl::base::SpaceInformation *space_info);
    DARTGeometricStateValidityChecker(
        ::ompl::base::SpaceInformationPtr const &space_info);

    virtual bool isValid(::ompl::base::State const *state) const;

private:
    DARTGeometricStateSpace *state_space_;
};

} // namespace r3::ompl
} // namespace r3

#endif
