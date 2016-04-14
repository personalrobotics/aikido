#ifndef AIKIDO_GEODESIC_DISTANCE_H_
#define AIKIDO_GEODESIC_DISTANCE_H_

#include <aikido/distance/DistanceMetric.hpp>
#include <aikido/statespace/SO3StateSpace.hpp>

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

  /// Computes distance between two states as the angle between the
  /// two quaternions represented by the states.
  double distance(const statespace::StateSpace::State* _state1,
                  const statespace::StateSpace::State* _state2) const override;

  /// Computes the slerp that interpolates between the quaternions represented
  /// in the states
  void interpolate(const statespace::StateSpace::State* _from,
                   const statespace::StateSpace::State* _to, double _t,
                   statespace::StateSpace::State* _state) const override;

private:
  std::shared_ptr<statespace::SO3StateSpace> mStateSpace;
};
};
}
#endif
