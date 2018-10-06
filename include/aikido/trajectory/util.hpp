#ifndef AIKIDO_TRAJECTORY_UTIL_HPP_
#define AIKIDO_TRAJECTORY_UTIL_HPP_

#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <boost/program_options.hpp>
#include <ros/ros.h>

namespace aikido {
namespace trajectory {

/// Create a timed spline with only two waypoints with start/end velocities
std::unique_ptr<aikido::trajectory::Spline> createTimedSplineTrajectory(
    const Eigen::VectorXd& startPosition,
    const Eigen::VectorXd& endPosition,
    const Eigen::VectorXd& startVelocity,
    const Eigen::VectorXd& endVelocity,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    aikido::statespace::ConstStateSpacePtr stateSpace,
    double startTime = 0.);

std::unique_ptr<aikido::trajectory::Spline> createTimedSplineTrajectory(
    const aikido::trajectory::Interpolated& interpolated,
    const Eigen::VectorXd& startVelocity,
    const Eigen::VectorXd& endVelocity,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration);

double findClosetStateOnTrajectory(
    const aikido::trajectory::Trajectory* traj,
    const Eigen::VectorXd& config,
    double timeStep = 0.01);

std::unique_ptr<aikido::trajectory::Spline> concatenate(
    const aikido::trajectory::Spline& traj1,
    const aikido::trajectory::Spline& traj2);

std::unique_ptr<aikido::trajectory::Spline> createPartialTrajectory(
    const aikido::trajectory::Spline& traj, double partialStartTime);

void printStateWithTime(
    double t,
    std::size_t dimension,
    Eigen::VectorXd& stateVec,
    Eigen::VectorXd& velocityVec,
    std::ofstream& cout);

void dumpSplinePhasePlot(
    const aikido::trajectory::Spline& spline,
    const std::string& filename,
    double timeStep);

}
}

#endif
