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

  /// Computes distance between two states as the weighted sum
  ///  of distances between their matching subcomponents.
  virtual double distance(
      const aikido::statespace::StateSpace::State* _state1,
      const aikido::statespace::StateSpace::State* _state2) const override;

  /// Computes the state that lies at time t in [0, 1] on the segment
  /// that connects from state to to state. This state is computed
  /// by interpolating each subcomponent individually.
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
