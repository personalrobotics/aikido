#include "../WeightedDistanceMetric.hpp"
#include "../AngularDistanceMetric.hpp"
#include "../GeodesicDistanceMetric.hpp"
#include "../EuclideanDistanceMetric.hpp"
#include "../../statespace/SO2StateSpace.hpp"
#include "../../statespace/SO3StateSpace.hpp"
#include "../../statespace/RealVectorStateSpace.hpp"
#include "../../statespace/CompoundStateSpace.hpp"

namespace aikido
{
namespace distance
{
namespace detail
{
using std::make_shared;
using Ptr = std::shared_ptr<DistanceMetric>;

//=============================================================================
template <class StateSpaceType>
struct createDistanceMetricFor_impl {
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<aikido::statespace::RealVectorStateSpace> {
  static Ptr create(
      std::shared_ptr<aikido::statespace::RealVectorStateSpace> _sspace)
  {
    return make_shared<EuclideanDistanceMetric>(_sspace);
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<aikido::statespace::SO2StateSpace> {
  static Ptr create(std::shared_ptr<aikido::statespace::SO2StateSpace> _sspace)
  {
    return make_shared<AngularDistanceMetric>(_sspace);
  }
};

//=============================================================================
template <>
struct createDistanceMetricFor_impl<aikido::statespace::SO3StateSpace> {
  static Ptr create(std::shared_ptr<aikido::statespace::SO3StateSpace> _sspace)
  {
    return make_shared<GeodesicDistanceMetric>(_sspace);
  }
};

//=============================================================================
template <class... Args>
struct ForOneOf {
};

template <>
struct ForOneOf<> {
  static Ptr create(std::shared_ptr<aikido::statespace::StateSpace> _sspace)
  {
    return nullptr;
  }
};

template <class Arg, class... Args>
struct ForOneOf<Arg, Args...> {
  static Ptr create(std::shared_ptr<aikido::statespace::StateSpace> _sspace)
  {
      auto sspace = std::dynamic_pointer_cast<Arg>(_sspace);

      if (sspace) {
          return createDistanceMetricFor_impl<Arg>::create(sspace);
    } else {
      return ForOneOf<Args...>::create(_sspace);
    }
  }
};

//=============================================================================
using createDistanceMetricFor_wrapper =
    ForOneOf<aikido::statespace::SO2StateSpace,
             aikido::statespace::SO3StateSpace,
             aikido::statespace::RealVectorStateSpace,
             aikido::statespace::CompoundStateSpace>;

//=============================================================================
template <>
struct createDistanceMetricFor_impl<aikido::statespace::CompoundStateSpace> {
  static Ptr create(
      std::shared_ptr<aikido::statespace::CompoundStateSpace> _sspace)
  {
    std::vector<std::shared_ptr<DistanceMetric> > metrics;
    metrics.resize(_sspace->getNumStates());
    for (size_t i = 0; i < _sspace->getNumStates(); ++i) {
      metrics[i] = 
          createDistanceMetricFor_wrapper::create(_sspace->getSubSpacePtr<>(i));
    }
    return make_shared<WeightedDistanceMetric>(_sspace, metrics);
  }
};

}  // detail

//=============================================================================
template <class StateSpaceType>
std::shared_ptr<DistanceMetric> createDistanceMetricFor(
    std::shared_ptr<StateSpaceType> _sspace)
{
  return detail::createDistanceMetricFor_impl<StateSpaceType>::create(_sspace);
}
}
}
