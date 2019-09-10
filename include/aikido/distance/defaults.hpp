#ifndef AIKIDO_DISTANCE_DISTANCEMETRICDEFAULTS_HPP_
#define AIKIDO_DISTANCE_DISTANCEMETRICDEFAULTS_HPP_

#include "aikido/distance/DistanceMetric.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace distance {
/// Creates a DistanceMetric that is appropriate for the statespace of type
/// Space
/// \param _sspace The StateSpace the distance metric will operator on
template <class Space>
std::unique_ptr<DistanceMetric> createDistanceMetricFor(
    std::shared_ptr<Space> _sspace);

/// Creates a DistanceMetric that is appropriate for the statespace.
/// \param _sspace The StateSpace the distance metric will operator on
std::unique_ptr<DistanceMetric> createDistanceMetric(
    statespace::ConstStateSpacePtr _sspace);
} // namespace distance
} // namespace aikido

#include "detail/defaults-impl.hpp"

#endif
