#include "aikido/distance/RnEuclidean.hpp"

namespace aikido {
namespace distance {

template class REuclidean<0>;

template class REuclidean<1>;

template class REuclidean<2>;

template class REuclidean<3>;

template class REuclidean<6>;

template class REuclidean<Eigen::Dynamic>;

} // namespace distance
} // namespace aikido
