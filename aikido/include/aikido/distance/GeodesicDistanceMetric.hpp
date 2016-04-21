#ifndef AIKIDO_DISTANCE_GEODESICDISTANCEMETRIC_HPP_
#define AIKIDO_DISTANCE_GEODESICDISTANCEMETRIC_HPP_

#include "DistanceMetric.hpp"
#include "../statespace/SO3StateSpace.hpp"

namespace aikido
{
namespace distance
{
/// Implements a distance metric on SO(3)
class GeodesicDistanceMetric : public DistanceMetric
{
public:
  /// Constructor.
  explicit GeodesicDistanceMetric(
      std::shared_ptr<statespace::SO3StateSpace> _space);

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override;

  /// Computes distance (in radians) between the two states
  double distance(const statespace::StateSpace::State* _state1,
                  const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::SO3StateSpace> mStateSpace;
};
};
}
#endif
