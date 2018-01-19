#ifndef AIKIDO_TRAJECTORY_TRAJECTORY_HPP_
#define AIKIDO_TRAJECTORY_TRAJECTORY_HPP_

#include <Eigen/Core>
#include "aikido/statespace/StateSpace.hpp"
#include "aikido/statespace/smart_pointer.hpp"
#include "aikido/trajectory/TrajectoryMetadata.hpp"

namespace aikido {
namespace trajectory {

/// Time-parameterized path in a \c StateSpace. The parameterization, number of
/// derivatives available, and continuity of this trajectory is defined by the
/// concrete implementation of this class. The interpretation of the time
/// parameter is also implementation defined: it may represent an actual time
/// time or some other value (e.g. arc length under a distance metric).
class Trajectory
{
public:
  virtual ~Trajectory() = default;

  /// Gets the \c StateSpace that this trajectory is defined in.
  ///
  /// \return state space this trajectory is defined in.
  virtual statespace::ConstStateSpacePtr getStateSpace() const = 0;

  /// Gets an upper bound on the number of non-zero derivatives available in
  /// this parameterization. Note that \c evaluateDerivative may return zero
  /// before this value for some trajectories.
  ///
  /// \return upper bound on the number of non-zero derivatives
  virtual std::size_t getNumDerivatives() const = 0;

  /// Duration of the trajectory. Note that \c getStartTime() may not be zero.
  ///
  /// \return duration of the trajectory
  virtual double getDuration() const = 0;

  /// Time at which the trajectory starts. This may not be zero.
  ///
  /// \return time at which the trajectory starts
  virtual double getStartTime() const = 0;

  /// Time at which the trajectory ends. This may not be \c getDuration() if
  /// \c getStartTime() is not zero.
  ///
  /// \return time at which the trajectory ends
  virtual double getEndTime() const = 0;

  /// Evaluates the state of the trajectory at time \c _t and store the result
  /// in a \c _state allocated by \c getStateSpace(). The output of this
  /// function is implementation-defined if \c _t is not between
  /// \c getStartTime() and \c getEndTime().
  ///
  /// \param _t time parameter
  /// \param[out] _state output state of the trajectory at time \c _t
  virtual void evaluate(
      double _t, statespace::StateSpace::State* _state) const = 0;

  /// Evaluates the derivative of the trajectory at time \c _t. The
  /// \c _tangentVector is defined in the local frame (i.e. "body frame") and
  /// is implementation-defined if not between \c getStartTime() and
  /// \c getEndTime(). Derivatives of order higher than \c getNumDerivatives
  /// are guaranteed to be zero.
  ///
  /// \param _t time parameter
  /// \param _derivative order of derivative
  /// \param[out] _tangentVector output tangent vector in the local frame
  virtual void evaluateDerivative(
      double _t, int _derivative, Eigen::VectorXd& _tangentVector) const = 0;

  /// Trajectory metadata
  TrajectoryMetadata metadata;
};

using TrajectoryPtr = std::shared_ptr<Trajectory>;
using ConstTrajectoryPtr = std::shared_ptr<const Trajectory>;

} // namespace trajectory
} // namespace aikido

#endif // ifndef AIKIDO_TRAJECTORY_TRAJECTORY_HPP_
