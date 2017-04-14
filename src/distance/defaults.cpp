#include <aikido/distance/defaults.hpp>

namespace aikido {
namespace distance {

//=============================================================================
std::unique_ptr<DistanceMetric> createDistanceMetric(
    statespace::StateSpacePtr _sspace)
{
  if (_sspace == nullptr)
    throw std::invalid_argument("_sspace is null.");

  return util::DynamicCastFactory<detail::createDistanceMetricFor_impl,
                                  util::DynamicCastFactory_shared_ptr,
                                  statespace::StateSpace,
                                  detail::SupportedStateSpaces>::
      create(_sspace);
}

} // namespace distance
} // namespace aikido
