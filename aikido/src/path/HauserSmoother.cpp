#include <boost/format.hpp>
#include <aikido/path/GeometricPath.h>
#include <aikido/path/SplineTrajectory.h>
#include "external/hauser_parabolic_smoother/DynamicPath.h"

using aikido::path::GeometricPath;
using aikido::path::TrajectoryPtr;

static ParabolicRamp::Vector toHauserVector(Eigen::VectorXd const &u)
{
  ParabolicRamp::Vector v(u.size());
  for (size_t i = 0; i < u.size(); ++i) {
    v[i] = u[i];
  }
  return v;
}

static ParabolicRamp::DynamicPath convertPathToHauserPath(
  GeometricPath const &path)
{
  using boost::format;
  using boost::str;
  using dart::dynamics::DegreeOfFreedom;

  static double const velEpsilon = 1e-3;
  static double const accEpsilon = 1e-3;

  ParabolicRamp::Vector posMin(path.num_dofs()),
                        posMax(path.num_dofs()),
                        velMax(path.num_dofs()),
                        accMax(path.num_dofs());

  for (size_t i = 0; i < path.num_dofs(); ++i) {
    DegreeOfFreedom const *dof = path.dofs()[i].get();

    // Position limits.
    posMin[i] = dof->getPositionLowerLimit();
    posMax[i] = dof->getPositionUpperLimit();

    // Velocity limits.
    double const velMaxNeg = dof->getVelocityLowerLimit();
    double const velMaxPos = dof->getVelocityUpperLimit();
    velMax[i] = std::min(-velMaxNeg, velMaxPos);

    if (std::abs(velMaxPos + velMaxNeg) > velEpsilon) {
      std::cerr << (format(
        "Warning: Velocity limits on DOF '%s' are assymmetric; %f != %f."
        " Using a conservative limit of %f.")
          % -velMaxNeg % velMaxPos % velMax[i]) << std::endl;
    }

    // Acceleration limits.
    double const accMaxNeg = dof->getAccelerationLowerLimit();
    double const accMaxPos = dof->getAccelerationUpperLimit();
    accMax[i] = std::min(-accMaxNeg, accMaxPos);

    if (std::abs(accMaxPos + accMaxNeg) > accEpsilon) {
      std::cerr << (format(
        "Warning: Acceleration limits on DOF '%s' are assymmetric; %f != %f."
        " Using a conservative limit of %f.")
          % -accMaxNeg % accMaxPos % accMax[i]) << std::endl;
    }
  }

  std::vector<ParabolicRamp::Vector> milestones(
    path.num_waypoints(), ParabolicRamp::Vector(path.num_dofs()));

  for (size_t iwaypoint = 0; iwaypoint < path.num_waypoints(); ++iwaypoint) {
    for (size_t idof = 0; idof < path.num_dofs(); ++idof) {
      milestones[iwaypoint][idof] = path.matrix()(iwaypoint, idof);
    }
  }

  ParabolicRamp::DynamicPath hauserPath;
  hauserPath.Init(velMax, accMax);
  hauserPath.SetJointLimits(posMin, posMax);
  hauserPath.SetMilestones(milestones);
  return hauserPath;
}

static TrajectoryPtr convertHauserPathToTrajectory(
  GeometricPath const &path)
{
  throw std::runtime_error("not implemented");
}

void HauserShortcut(GeometricPath const &path)
{
  ParabolicRamp::DynamicPath hauserPath = convertPathToHauserPath(path);

#if 0
  void Init(const Vector& velMax,const Vector& accMax);
  void SetJointLimits(const Vector& qMin,const Vector& qMax);
#endif
}
