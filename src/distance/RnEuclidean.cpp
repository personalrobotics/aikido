#include <aikido/distance/RnEuclidean.hpp>

namespace aikido {
namespace distance {

template
class RnEuclidean<0>;

template
class RnEuclidean<1>;

template
class RnEuclidean<2>;

template
class RnEuclidean<3>;

template
class RnEuclidean<6>;

template
class RnEuclidean<Eigen::Dynamic>;

} // namespace distance
} // namespace aikido
