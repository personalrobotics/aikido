#ifndef AIKIDO_STATESPACE_GEODESICINTERPOLATOR_HPP_
#define AIKIDO_STATESPACE_GEODESICINTERPOLATOR_HPP_
#include "StateSpace.hpp"
#include "Interpolator.hpp"

namespace aikido {
namespace statespace {

class GeodesicInterpolator : public Interpolator
{
public:
  explicit GeodesicInterpolator(statespace::StateSpacePtr _stateSpace);

  virtual ~GeodesicInterpolator() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Gets the tangent vector that defines the geodesic, scaled such that:
  ///   _to = _from * expMap(tangentVector)
  Eigen::VectorXd getTangentVector(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to) const;

  // Documentation inherited.
  void interpolate(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to, double _alpha,
    statespace::StateSpace::State* _state) const override;

private:
  statespace::StateSpacePtr mStateSpace;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_GEODESICINTERPOLATOR_HPP_
