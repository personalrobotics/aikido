#ifndef AIKIDO_OMPL_DARTGEOMETRICSTATEVALIDITYCHECKER_H_
#define AIKIDO_OMPL_DARTGEOMETRICSTATEVALIDITYCHECKER_H_
#include <ompl/base/StateValidityChecker.h>

namespace aikido {
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

} // namespace ompl
} // namespace aikido

#endif // AIKIDO_OMPL_DARTGEOMETRICSTATEVALIDITYCHECKER_H_
