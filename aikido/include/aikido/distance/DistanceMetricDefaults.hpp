#ifndef AIKIDO_DISTANCE_DISTANCEDEFAULTS_H_
#define AIKIDO_DISTANCE_DISTANCEDEFAULTS_H_

#include "DistanceMetric.hpp"

namespace aikido
{
namespace distance
{
template <class Space>
std::unique_ptr<DistanceMetric> createDistanceMetricFor(
    std::shared_ptr<Space> _sspace);

}
}

#include "detail/DistanceMetricDefaults.hpp"

#endif
