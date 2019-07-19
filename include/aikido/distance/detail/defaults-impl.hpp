#include "aikido/common/memory.hpp"

#include "../../common/metaprogramming.hpp"
#include "../../statespace/CartesianProduct.hpp"
#include "../../statespace/Rn.hpp"
#include "../../statespace/SE2.hpp"
#include "../../statespace/SO2.hpp"
#include "../../statespace/SO3.hpp"
#include "../CartesianProductWeighted.hpp"
#include "../RnEuclidean.hpp"
#include "../SE2Weighted.hpp"
#include "../SO2Angular.hpp"
#include "../SO3Angular.hpp"

namespace aikido {
namespace distance {
namespace detail {

using Ptr = std::unique_ptr<DistanceMetric>;

//==============================================================================
template <class Space>
struct createDistanceMetricFor_impl
{
  // Nothing defined
};

//==============================================================================
template <>
struct createDistanceMetricFor_impl<const statespace::R0>
{
  static Ptr create(std::shared_ptr<const statespace::R0> _sspace)
  {
    return ::aikido::common::make_unique<R0Euclidean>(std::move(_sspace));
  }
};

//==============================================================================
template <>
struct createDistanceMetricFor_impl<const statespace::R1>
{
  static Ptr create(std::shared_ptr<const statespace::R1> _sspace)
  {
    return ::aikido::common::make_unique<R1Euclidean>(std::move(_sspace));
  }
};

//==============================================================================
template <>
struct createDistanceMetricFor_impl<const statespace::R2>
{
  static Ptr create(std::shared_ptr<const statespace::R2> _sspace)
  {
    return ::aikido::common::make_unique<R2Euclidean>(std::move(_sspace));
  }
};

//==============================================================================
template <>
struct createDistanceMetricFor_impl<const statespace::R3>
{
  static Ptr create(std::shared_ptr<const statespace::R3> _sspace)
  {
    return ::aikido::common::make_unique<R3Euclidean>(std::move(_sspace));
  }
};

//==============================================================================
template <>
struct createDistanceMetricFor_impl<const statespace::R6>
{
  static Ptr create(std::shared_ptr<const statespace::R6> _sspace)
  {
    return ::aikido::common::make_unique<R6Euclidean>(std::move(_sspace));
  }
};

//==============================================================================
template <>
struct createDistanceMetricFor_impl<const statespace::SO2>
{
  static Ptr create(std::shared_ptr<const statespace::SO2> _sspace)
  {
    return ::aikido::common::make_unique<SO2Angular>(std::move(_sspace));
  }
};

//==============================================================================
template <>
struct createDistanceMetricFor_impl<const statespace::SO3>
{
  static Ptr create(std::shared_ptr<const statespace::SO3> _sspace)
  {
    return ::aikido::common::make_unique<SO3Angular>(std::move(_sspace));
  }
};

//==============================================================================
template <>
struct createDistanceMetricFor_impl<const statespace::CartesianProduct>
{
  static Ptr create(std::shared_ptr<const statespace::CartesianProduct> _sspace)
  {
    if (_sspace == nullptr)
      throw std::invalid_argument(
          "Cannot create distance metric for null statespace.");

    std::vector<std::shared_ptr<DistanceMetric> > metrics;
    metrics.reserve(_sspace->getNumSubspaces());

    for (std::size_t i = 0; i < _sspace->getNumSubspaces(); ++i)
    {
      auto subspace = _sspace->getSubspace<>(i);
      auto metric = createDistanceMetric(std::move(subspace));
      metrics.emplace_back(std::move(metric));
    }

    return ::aikido::common::make_unique<CartesianProductWeighted>(
        std::move(_sspace), std::move(metrics));
  }
};

//==============================================================================
template <>
struct createDistanceMetricFor_impl<const statespace::SE2>
{
  static Ptr create(std::shared_ptr<const statespace::SE2> _sspace)
  {
    return ::aikido::common::make_unique<SE2Weighted>(std::move(_sspace));
  }
};

//==============================================================================
using SupportedStateSpaces
    = common::type_list<const statespace::CartesianProduct,
                        const statespace::R0,
                        const statespace::R1,
                        const statespace::R2,
                        const statespace::R3,
                        const statespace::R6,
                        const statespace::SO2,
                        const statespace::SO3,
                        const statespace::SE2>;

} // namespace detail

//==============================================================================
template <class Space>
std::unique_ptr<DistanceMetric> createDistanceMetricFor(
    std::shared_ptr<Space> _sspace)
{
  if (_sspace == nullptr)
    throw std::invalid_argument("_sspace is null.");

  return detail::createDistanceMetricFor_impl<const Space>::create(
      std::move(_sspace));
}

} // namespace distance
} // namespace aikido
