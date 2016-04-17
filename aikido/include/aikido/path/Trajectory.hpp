#ifndef AIKIDO_PATH_TRAJECTORY_H_
#define AIKIDO_PATH_TRAJECTORY_H_
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include "../statespace/StateSpace.hpp"

namespace aikido
{
namespace path
{
class Trajectory
{
public:
  virtual ~Trajectory() = default;

  /// Return the StateSpace this Trajectory is defined on
  virtual aikido::statespace::StateSpacePtr getStateSpace() const = 0;

  /// The number of non-zero value derivatives availabler
  virtual int getNumDerivatives() const = 0;

  /// The duration of the trajectory
  virtual double getDuration() const = 0;

  /// The time on the first waypoint of the trajectory
  virtual double getStartTime() const = 0;

  /// The time on the last waypoint of the trajectory
  virtual double getEndTime() const = 0;

  /// Compute the state that should be achieved at time t when following
  ///  the trajectory.
  virtual void evaluate(
      double _t, aikido::statespace::StateSpace::State *_state) const = 0;

  /// Compute the derivative of the trajectory at time t.
  virtual Eigen::VectorXd evaluate(double _t, int _derivative) const = 0;
};

using TrajectoryPtr = boost::shared_ptr<Trajectory>;
using ConstTrajectoryPtr = boost::shared_ptr<const Trajectory>;

}  // namespace path
}  // namespace aikido

#endif  // ifndef AIKIDO_PATH_TRAJECTORY_H_
