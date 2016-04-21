#ifndef AIKIDO_DISTANCE_ANGULARDISTANCEMETRIC_HPP_
#define AIKIDO_DISTANCE_ANGULARDISTANCEMETRIC_HPP_

#include "DistanceMetric.hpp"
#include "../statespace/SO2StateSpace.hpp"

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

private:
  std::shared_ptr<statespace::SO2StateSpace> mStateSpace;
};
};
}
#endif
