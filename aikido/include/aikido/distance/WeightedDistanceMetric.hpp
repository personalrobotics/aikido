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
  /// Constructor. By default the weights on all subspaces is 1.
  WeightedDistanceMetric(std::shared_ptr<statespace::CompoundStateSpace> _space,
                   std::vector<DistanceMetricPtr> _metrics);

  /// Constructor.
  WeightedDistanceMetric(std::shared_ptr<statespace::CompoundStateSpace> _space,
                   std::vector<DistanceMetricPtr> _metrics,
                   std::vector<double> _weights);

  /// Computes distance between two angles. (return value between 0 and pi)
  virtual double distance(
      const aikido::statespace::StateSpace::State* _state1,
      const aikido::statespace::StateSpace::State* _state2) const override;

  /// Computes the state that lies at time t in [0, 1] on the segment
  /// that connects from state to to state. The memory location of state
  /// is not required to be different from the memory of either from or to.
  virtual void interpolate(
      const aikido::statespace::StateSpace::State* _from,
      const aikido::statespace::StateSpace::State* _to, const double _t,
      aikido::statespace::StateSpace::State* _state) const override;

private:
  std::shared_ptr<statespace::CompoundStateSpace> mStateSpace;
  std::vector<DistanceMetricPtr> mMetrics;
  std::vector<double> mWeights;
};
}
}
#endif
