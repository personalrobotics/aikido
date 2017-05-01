#ifndef PARABOLIC_UTIL_HPP_
#define PARABOLIC_UTIL_HPP_

#include <memory>
#include <Eigen/Dense>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include "DynamicPath.h"

namespace aikido {
namespace planner {
namespace parabolic {

ParabolicRamp::Vector toVector(const Eigen::VectorXd& _x);

Eigen::VectorXd toEigen(const ParabolicRamp::Vector& _x);

void evaluateAtTime(ParabolicRamp::DynamicPath& _path, double _t,
    Eigen::VectorXd& _position, Eigen::VectorXd& _velocity);

bool checkStateSpace(const aikido::statespace::StateSpace* _stateSpace);

std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
    const aikido::trajectory::Interpolated& _inputTrajectory);

std::unique_ptr<aikido::trajectory::Spline> convertToSpline(
        ParabolicRamp::DynamicPath* _inputPath,
        double _startTime,
        aikido::statespace::StateSpacePtr _stateSpace);

std::unique_ptr<ParabolicRamp::DynamicPath>
    convertToDynamicPath(aikido::trajectory::Spline* _inputTrajectory,
                         double& _startTime,
                         const Eigen::VectorXd& _maxVelocity,
                         const Eigen::VectorXd& _maxAcceleration);


} // namespace parabolic
} // namespace planner
} // namespace aikido

#endif // PARABOLIC_UTIL_HPP_
