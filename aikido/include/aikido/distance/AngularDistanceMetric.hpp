#ifndef AIKIDO_ANGULAR_DISTANCE_H_
#define AIKIDO_ANGULAR_DISTANCE_H_

#include <aikido/distance/DistanceMetric.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>

namespace aikido
{
namespace distance
{
/// Computes the shortest distance between two angles in SO(2)
class AngularDistanceMetric : public DistanceMetric
{
public:
  /// Constructor.
  explicit AngularDistanceMetric(
      std::shared_ptr<statespace::SO2StateSpace> _space);

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override;

  /// Computes distance between two angles. (return value between 0 and pi)
  double distance(const statespace::StateSpace::State* _state1,
                  const statespace::StateSpace::State* _state2) const override;

  /// Computes an interpolated angle as (1-_t)*from + _t*to.
  void interpolate(const statespace::StateSpace::State* _from,
                   const statespace::StateSpace::State* _to, double _t,
                   statespace::StateSpace::State* _state) const override;

private:
  std::shared_ptr<statespace::SO2StateSpace> mStateSpace;
};
};
}
#endif
