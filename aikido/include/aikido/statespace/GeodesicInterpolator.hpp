#ifndef AIKIDO_STATESPACE_GEODESICINTERPOLATOR_HPP_
#define AIKIDO_STATESPACE_GEODESICINTERPOLATOR_HPP_
#include "StateSpace.hpp"
#include "Interpolator.hpp"

namespace aikido {
namespace statespace {

/// Interpolate by parallel transport along a geodesic between two states in a
/// Lie group with an affine connection. The geodesic is defined by a single
/// tangent vector that remains constant during interpolation.
class GeodesicInterpolator : public Interpolator
{
public:
  /// Constructs a \c GeodesicInterpolator for \c _stateSpace.
  ///
  /// \param _stateSpace used for interpolation
  explicit GeodesicInterpolator(statespace::StateSpacePtr _stateSpace);

  virtual ~GeodesicInterpolator() = default;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  size_t getNumDerivatives() const override;

  /// Gets the tangent vector that defines the geodesic, scaled such that:
  ///
  /// \code
  /// interpolate(alpha) = _from * expMap(alpha * tangentVector)
  /// \endcode
  ///
  /// \param _from start state in \c getStateSpace()
  /// \param _from start state in \c getStateSpace()
  /// \return tangent vector that defines the geodesic
  Eigen::VectorXd getTangentVector(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to) const;

  // Documentation inherited.
  void interpolate(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to, double _alpha,
    statespace::StateSpace::State* _state) const override;

  // Documentation inherited.
  Eigen::VectorXd getDerivative(
    const statespace::StateSpace::State* _from,
    const statespace::StateSpace::State* _to,
    size_t _derivative, double _alpha) const override;

private:
  statespace::StateSpacePtr mStateSpace;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_GEODESICINTERPOLATOR_HPP_
