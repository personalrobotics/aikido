#include "aikido/distance/defaults.hpp"

namespace aikido {
namespace distance {

//==============================================================================
std::unique_ptr<DistanceMetric> createDistanceMetric(
    statespace::ConstStateSpacePtr _sspace)
{
  if (_sspace == nullptr)
    throw std::invalid_argument("_sspace is null.");

  return common::DynamicCastFactory<
      detail::createDistanceMetricFor_impl,
      common::DynamicCastFactory_shared_ptr,
      const statespace::StateSpace,
      detail::SupportedStateSpaces>::create(_sspace);
}

} // namespace distance
} // namespace aikido
