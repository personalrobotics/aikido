#ifndef AIKIDO_DISTANCE_METASKELETONDISTANCE_H_
#define AIKIDO_DISTANCE_METASKELETONDISTANCE_H_

#include "DistanceMetric.hpp"

namespace aikido
{
namespace distance
{
template <class StateSpaceType>
std::shared_ptr<DistanceMetric> createDistanceMetricFor(
    std::shared_ptr<StateSpaceType> _sspace);
}
}

#include "detail/DistanceMetricDefaults.hpp"

#endif
