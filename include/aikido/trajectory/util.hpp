#ifndef AIKIDO_TRAJECTORY_UTIL_HPP_
#define AIKIDO_TRAJECTORY_UTIL_HPP_

#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>

namespace aikido {
namespace trajectory {

double findClosetStateOnTrajectory(
    const aikido::trajectory::Trajectory* traj,
    const Eigen::VectorXd& config,
    double timeStep = 0.01);

std::unique_ptr<aikido::trajectory::Spline> concatenate(
    const aikido::trajectory::Spline& traj1,
    const aikido::trajectory::Spline& traj2);

std::unique_ptr<aikido::trajectory::Spline> createPartialTrajectory(
    const aikido::trajectory::Spline& traj, double partialStartTime);

} // namespace trajectory
} // namespace aikido

#endif
