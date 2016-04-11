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
  AngularDistanceMetric(std::shared_ptr<statespace::SO2StateSpace> _space);

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
  std::shared_ptr<statespace::SO2StateSpace> mStateSpace;
};
};
}
#endif
