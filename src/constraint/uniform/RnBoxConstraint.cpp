#include <aikido/constraint/uniform/RnBoxConstraint.hpp>

namespace aikido {
namespace constraint {

template
class RnBoxConstraint<0>;

template
class RnBoxConstraint<1>;

template
class RnBoxConstraint<2>;

template
class RnBoxConstraint<3>;

template
class RnBoxConstraint<6>;

template
class RnBoxConstraint<Eigen::Dynamic>;

} // namespace statespace
} // namespace aikido
