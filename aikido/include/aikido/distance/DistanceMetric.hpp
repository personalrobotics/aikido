#ifndef AIKIDO_DISTANCE_DISTANCEMETRIC_HPP_
#define AIKIDO_DISTANCE_DISTANCEMETRIC_HPP_

#include "../statespace/StateSpace.hpp"

namespace aikido
{
namespace distance
{
/// Implements a distance metric defined on a StateSpace
class DistanceMetric
{
public:
  /// Destructor
  virtual ~DistanceMetric() = default;

  /// Get the StateSpace associated with this metric
  virtual statespace::StateSpacePtr getStateSpace() const = 0;

  /// Computes distance between two states. This function satisfies
  /// the properties of a metric:
  /// (1) distance(s1, s2) >= 0
  /// (2) distance(s1, s2) = 0 implies s1 == s2
  /// (3) distance(s1, s2) == distance(s2, s1)
  /// (4) distance(s1, s3) <= distance(s1, s2) + distance(s2, s3)
  virtual double distance(
      const statespace::StateSpace::State* _state1,
      const statespace::StateSpace::State* _state2) const = 0;
};

using DistanceMetricPtr = std::shared_ptr<DistanceMetric>;
}
}
#endif
