#ifndef AIKIDO_WEIGHTED_DISTANCE_H_
#define AIKIDO_WEIGHTED_DISTANCE_H_

#include <aikido/distance/DistanceMetric.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>

namespace aikido
{
namespace distance
{
class WeightedDistanceMetric : public DistanceMetric
{
public:
  /// Constructor. Default the weights applied to each subspace to 1.
  WeightedDistanceMetric(std::shared_ptr<statespace::CompoundStateSpace> _space,
                         std::vector<DistanceMetricPtr> _metrics);

  /// Constructor.
  WeightedDistanceMetric(std::shared_ptr<statespace::CompoundStateSpace> _space,
                         std::vector<DistanceMetricPtr> _metrics,
                         std::vector<double> _weights);

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override;

  /// Computes distance between two states as the weighted sum
  ///  of distances between their matching subcomponents.
  double distance(const statespace::StateSpace::State* _state1,
                  const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::CompoundStateSpace> mStateSpace;
  std::vector<DistanceMetricPtr> mMetrics;
  std::vector<double> mWeights;
};
}
}
#endif
