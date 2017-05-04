#ifndef AIKIDO_STATESPACE_INTERPOLATOR_HPP_
#define AIKIDO_STATESPACE_INTERPOLATOR_HPP_
#include <memory>
#include "../statespace/StateSpace.hpp"

namespace aikido {
namespace statespace {

/// Method of interpolating between two states in a \c StateSpace.
class Interpolator
{
public:
  virtual ~Interpolator() = default;

  /// Gets the \c StateSpace on which this \c Interpolator operates.
  virtual statespace::StateSpacePtr getStateSpace() const = 0;

  /// Upper bound on the number of non-zero derivatives.
  virtual size_t getNumDerivatives() const = 0;

  /// Computes the state that lies at path parameter \c _alpha along the path
  /// that connects \c _from to \c _to. By definition \c interpolate(0) is
  /// \c _from and \c interpolate(1) is \c _to. The memory location of \c
  /// _state must differ from the memory locations of \c _from and \c _to.
  ///
  /// \param _from start state in \c getStateSpace()
  /// \param _to end state in \c getStateSpace()
  /// \param _alpha path parameter in the range [0, 1]
  /// \param[out] _state output interpolated state
  virtual void interpolate(
      const statespace::StateSpace::State* _from,
      const statespace::StateSpace::State* _to,
      double _alpha,
      statespace::StateSpace::State* _state) const = 0;

  /// Computes the <tt>_derivative</tt>-th derivative of the path at path
  /// parameter \c _alpha between \c _from and \c _to. The output is an element
  /// of the tangent space in the local (i.e. "body") frame.
  ///
  /// \param _from start state in \c getStateSpace()
  /// \param _to end state in \c getStateSpace()
  /// \param _derivative order of the derivative to compute
  /// \param _alpha path parameter in the range [0, 1]
  /// \param[out] _tangentVector output element of the tangent space
  virtual void getDerivative(
      const statespace::StateSpace::State* _from,
      const statespace::StateSpace::State* _to,
      size_t _derivative,
      double _alpha,
      Eigen::VectorXd& _tangentVector) const = 0;
};

using InterpolatorPtr = std::shared_ptr<Interpolator>;

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_INTERPOLATOR_HPP_
