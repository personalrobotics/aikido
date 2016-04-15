#ifndef AIKIDO_STATESPACE_INTERPOLATOR_HPP_
#define AIKIDO_STATESPACE_INTERPOLATOR_HPP_
#include <memory>
#include "../statespace/StateSpace.hpp"

namespace aikido {
namespace statespace {

class Interpolator
{
public:
  virtual ~Interpolator() = default;

  virtual statespace::StateSpacePtr getStateSpace() const = 0;

  /// Upper bound on the number of non-zero derivatives.
  virtual size_t getNumDerivatives() const = 0;

  /// Computes the state that lies at ratio _alpha in [0, 1] on the segment
  /// that connects from state to to state. The memory location of state
  /// is not required to be different from the memory of either from or to.
  virtual void interpolate(
      const statespace::StateSpace::State* _from,
      const statespace::StateSpace::State* _to, double _alpha,
      statespace::StateSpace::State* _state) const = 0;

  /// Computes the derivative of the interpolant between _from and _to.
  virtual Eigen::VectorXd getDerivative(
      const statespace::StateSpace::State* _from,
      const statespace::StateSpace::State* _to,
      size_t _derivative, double _alpha) const = 0;
};

using InterpolatorPtr = std::shared_ptr<Interpolator>;

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_INTERPOLATOR_HPP_
