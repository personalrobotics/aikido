#include "aikido/constraint/uniform/RnBoxConstraint.hpp"

namespace aikido {
namespace constraint {
namespace uniform {

template class RBoxConstraint<0>;

template class RBoxConstraint<1>;

template class RBoxConstraint<2>;

template class RBoxConstraint<3>;

template class RBoxConstraint<6>;

template class RBoxConstraint<Eigen::Dynamic>;

} // namespace uniform
} // namespace constraint
} // namespace aikido
