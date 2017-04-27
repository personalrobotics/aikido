#include <aikido/constraint/uniform/RnBoxConstraint.hpp>

namespace aikido {
namespace constraint {

template
class RBoxConstraint<0>;

template
class RBoxConstraint<1>;

template
class RBoxConstraint<2>;

template
class RBoxConstraint<3>;

template
class RBoxConstraint<6>;

template
class RBoxConstraint<Eigen::Dynamic>;

} // namespace statespace
} // namespace aikido
