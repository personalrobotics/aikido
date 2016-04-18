#include "../../distance/DistanceMetric.hpp"
#include "../../statespace/SO2StateSpace.hpp"
#include "../../statespace/SO3StateSpace.hpp"
#include "../../statespace/RealVectorStateSpace.hpp"
#include "../../statespace/CompoundStateSpace.hpp"
#include <limits>

namespace aikido
{
namespace ompl
{
namespace detail
{
//=============================================================================
template <class Space>
struct getMaximumExtentFor_impl {
};

//=============================================================================
template <>
struct getMaximumExtentFor_impl<statespace::SO2StateSpace>
{
  static double create(std::shared_ptr<statespace::SO2StateSpace> _stateSpace,
                       std::shared_ptr<distance::DistanceMetric> _distMetric)
  {
    auto tmp1 = _stateSpace->createState();
    auto tmp2 = _stateSpace->createState();
    _stateSpace->setAngle(tmp1, M_PI);
    _stateSpace->setAngle(tmp2, 0);
    return _distMetric->distance(tmp1, tmp2);
  }
};

//=============================================================================
template <>
struct getMaximumExtentFor_impl<statespace::SO3StateSpace>
{
  static double create(std::shared_ptr<statespace::SO3StateSpace> _stateSpace,
                       std::shared_ptr<distance::DistanceMetric> _distMetric)
  {
      // TODO
  }
};

template <>
struct getMaximumExtentFor_impl<statespace::SO3StateSpace>
{
  static double create(std::shared_ptr<statespace::SO3StateSpace> _stateSpace,
                       std::shared_ptr<distance::DistanceMetric> _distMetric)
  {
      // TODO
  }
};

//=============================================================================
template <class... Args>
struct ForOneOf {
};

template <>
struct ForOneOf<> {
  static double create(
      std::shared_ptr<statespace::StateSpace> _sspace,
      std::shared_ptr<distance::DistanceMetric> _distanceMetric)
  {
    return std::numeric_limits<double>::infinity();
  }
};

template <class Arg, class... Args>
struct ForOneOf<Arg, Args...> {
  static double create(
      std::shared_ptr<statespace::StateSpace> _sspace,
      std::shared_ptr<distance::DistanceMetric> _distanceMetric)
  {
    auto sspace = std::dynamic_pointer_cast<Arg>(_sspace);

    if (sspace) {
      return getMaximumExtentFor_impl<Arg>::create(std::move(sspace),
                                                   std::move(_distanceMetric));
    } else {
        return ForOneOf<Args...>::create(_sspace, _distanceMetric);
    }
  }
};

//=============================================================================
using getMaximumExtentFor_wrapper =
    ForOneOf<statespace::SO2StateSpace, statespace::SO3StateSpace,
             statespace::RealVectorStateSpace>;
//    ForOneOf<statespace::SO2StateSpace, statespace::SO3StateSpace,
//             statespace::RealVectorStateSpace, statespace::CompoundStateSpace>;

}  // detail

//=============================================================================
template <class Space>
double getMaximumExtentFor(
    std::shared_ptr<Space> _stateSpace,
    std::shared_ptr<distance::DistanceMetric> _distanceMetric)
{
  return detail::getMaximumExtentFor_wrapper::create(
      std::move(_stateSpace), std::move(_distanceMetric));
}
}
}
