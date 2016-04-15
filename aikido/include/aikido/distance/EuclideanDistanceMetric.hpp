#ifndef AIKIDO_EUCLIDEAN_DISTANCE_METRIC_H_
#define AIKIDO_EUCLIDEAN_DISTANCE_METRIC_H_

#include <aikido/distance/DistanceMetric.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>

namespace aikido
{
namespace distance
{
/// Implements a Euclidean distance metric
class EuclideanDistanceMetric : public DistanceMetric
{
public:
  /// Constructor.
  explicit EuclideanDistanceMetric(
      std::shared_ptr<statespace::RealVectorStateSpace> _space);

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override;

  /// Computes euclidean distance between two states.
  double distance(const statespace::StateSpace::State* _state1,
                  const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::RealVectorStateSpace> mStateSpace;
};
}
}

#endif
