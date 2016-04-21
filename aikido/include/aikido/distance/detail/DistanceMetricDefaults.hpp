#include "../WeightedDistanceMetric.hpp"
#include "../AngularDistanceMetric.hpp"
#include "../GeodesicDistanceMetric.hpp"
#include "../EuclideanDistanceMetric.hpp"
#include "../../statespace/SO2StateSpace.hpp"
#include "../../statespace/SO3StateSpace.hpp"
#include "../../statespace/RealVectorStateSpace.hpp"
#include "../../statespace/CompoundStateSpace.hpp"
#include <dart/common/StlHelpers.h>

namespace aikido
{
namespace distance
{
namespace detail
{
using dart::common::make_unique;
using Ptr = std::unique_ptr<DistanceMetric>;

//=============================================================================
template <class Space>
struct createDistanceMetricFor_impl {
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::RealVectorStateSpace> {
  static Ptr create(std::shared_ptr<statespace::RealVectorStateSpace> _sspace)
  {
    return make_unique<EuclideanDistanceMetric>(std::move(_sspace));
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::SO2StateSpace> {
  static Ptr create(std::shared_ptr<statespace::SO2StateSpace> _sspace)
  {
    return make_unique<AngularDistanceMetric>(std::move(_sspace));
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::SO3StateSpace> {
  static Ptr create(std::shared_ptr<statespace::SO3StateSpace> _sspace)
  {
    return make_unique<GeodesicDistanceMetric>(std::move(_sspace));
  }
};

//=============================================================================
template <class... Args>
struct ForOneOf {
};

template <>
struct ForOneOf<> {
  static Ptr create(std::shared_ptr<statespace::StateSpace> _sspace)
  {
    throw std::invalid_argument(
        "Unrecognized state space. Unable to create distance metric.");
  }
};

template <class Arg, class... Args>
struct ForOneOf<Arg, Args...> {
  static Ptr create(std::shared_ptr<statespace::StateSpace> _sspace)
  {
    auto sspace = std::dynamic_pointer_cast<Arg>(_sspace);

    if (sspace) {
      return createDistanceMetricFor_impl<Arg>::create(std::move(sspace));
    } else {
      return ForOneOf<Args...>::create(_sspace);
    }
  }
};

//=============================================================================
using createDistanceMetricFor_wrapper =
    ForOneOf<statespace::SO2StateSpace, statespace::SO3StateSpace,
             statespace::RealVectorStateSpace, statespace::CompoundStateSpace>;

//=============================================================================
template <>
struct createDistanceMetricFor_impl<statespace::CompoundStateSpace> {
  static Ptr create(std::shared_ptr<statespace::CompoundStateSpace> _sspace)
  {
    if (_sspace == nullptr)
      throw std::invalid_argument(
          "Cannot create distance metric for null statespace.");

    std::vector<std::shared_ptr<DistanceMetric> > metrics;
    metrics.reserve(_sspace->getNumStates());
    for (size_t i = 0; i < _sspace->getNumStates(); ++i) {
      auto subspace = _sspace->getSubSpace<>(i);
      auto metric =
          createDistanceMetricFor_wrapper::create(std::move(subspace));
      metrics.emplace_back(std::move(metric));
    }
    return make_unique<WeightedDistanceMetric>(std::move(_sspace),
                                               std::move(metrics));
  }
};

}  // detail

//=============================================================================
template <class Space>
std::unique_ptr<DistanceMetric> createDistanceMetricFor(
    std::shared_ptr<Space> _sspace)
{
  return detail::createDistanceMetricFor_impl<Space>::create(
      std::move(_sspace));
}

//=============================================================================
std::unique_ptr<DistanceMetric> createDistanceMetric(
    statespace::StateSpacePtr _sspace)
{
  return detail::createDistanceMetricFor_wrapper::create(std::move(_sspace));
}
}
}
