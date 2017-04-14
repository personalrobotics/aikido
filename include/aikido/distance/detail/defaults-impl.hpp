#include "../Weighted.hpp"
#include "../SO2Angular.hpp"
#include "../SO3Angular.hpp"
#include "../RnEuclidean.hpp"
#include "../../statespace/SO2.hpp"
#include "../../statespace/SO3.hpp"
#include "../../statespace/Rn.hpp"
#include "../../statespace/CartesianProduct.hpp"
#include "../../util/metaprogramming.hpp"
#include <dart/common/StlHelpers.hpp>

namespace aikido {
namespace distance {
namespace detail {

using dart::common::make_unique;
using Ptr = std::unique_ptr<DistanceMetric>;

//=============================================================================
template <class Space>
struct createDistanceMetricFor_impl {};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::R0>
{
  static Ptr create(std::shared_ptr<statespace::R0> _sspace)
  {
    return make_unique<R0Euclidean>(std::move(_sspace));
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::R1>
{
  static Ptr create(std::shared_ptr<statespace::R1> _sspace)
  {
    return make_unique<R1Euclidean>(std::move(_sspace));
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::R2>
{
  static Ptr create(std::shared_ptr<statespace::R2> _sspace)
  {
    return make_unique<R2Euclidean>(std::move(_sspace));
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::R3>
{
  static Ptr create(std::shared_ptr<statespace::R3> _sspace)
  {
    return make_unique<R3Euclidean>(std::move(_sspace));
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::R6>
{
  static Ptr create(std::shared_ptr<statespace::R6> _sspace)
  {
    return make_unique<R6Euclidean>(std::move(_sspace));
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::SO2>
{
  static Ptr create(std::shared_ptr<statespace::SO2> _sspace)
  {
    return make_unique<SO2Angular>(std::move(_sspace));
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::SO3>
{
  static Ptr create(std::shared_ptr<statespace::SO3> _sspace)
  {
    return make_unique<SO3Angular>(std::move(_sspace));
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::CartesianProduct>
{
  static Ptr create(std::shared_ptr<statespace::CartesianProduct> _sspace)
  {
    if (_sspace == nullptr)
      throw std::invalid_argument(
          "Cannot create distance metric for null statespace.");

    std::vector<std::shared_ptr<DistanceMetric> > metrics;
    metrics.reserve(_sspace->getNumSubspaces());

    for (size_t i = 0; i < _sspace->getNumSubspaces(); ++i)
    {
      auto subspace = _sspace->getSubspace<>(i);
      auto metric = createDistanceMetric(std::move(subspace));
      metrics.emplace_back(std::move(metric));
    }

    return make_unique<Weighted>(
      std::move(_sspace), std::move(metrics));
  }
};

//=============================================================================
using SupportedStateSpaces = util::type_list<
    statespace::CartesianProduct,
    statespace::R0,
    statespace::R1,
    statespace::R2,
    statespace::R3,
    statespace::R6,
    statespace::SO2,
    statespace::SO3
  >;

} // namespace detail

//=============================================================================
template <class Space>
std::unique_ptr<DistanceMetric> createDistanceMetricFor(
    std::shared_ptr<Space> _sspace)
{
  if (_sspace == nullptr)
    throw std::invalid_argument("_sspace is null.");

  return detail::createDistanceMetricFor_impl<Space>
    ::create(std::move(_sspace));
}

} // namespace distance
} // namespace aikido
