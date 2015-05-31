#include <unordered_set>
#include <boost/format.hpp>
#include <r3/path/GeometricPath.h>

using boost::format;
using boost::str;
using dart::dynamics::DegreeOfFreedomPtr;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using r3::path::GeometricPath;


/*
 * GeometricPath::Waypoint
 */
std::vector<DegreeOfFreedomPtr> const &GeometricPath::Waypoint::dofs() const
{
  return path_->dofs_;
}

Eigen::VectorXd GeometricPath::Waypoint::positions() const
{
  return path_->waypoints_.row(index_);
}

GeometricPath::Waypoint::Waypoint(GeometricPath const *path, size_t index)
  : path_(path)
  , index_(index)
{
}


/*
 * GeometricPath
 */
GeometricPath::GeometricPath(
    std::vector<DegreeOfFreedomPtr> const &dofs, MatrixXd const &waypoints)
  : dofs_(dofs)
  , waypoints_(waypoints)
{
  using dart::dynamics::DegreeOfFreedom;

  if (waypoints_.cols() != dofs_.size()) {
    throw std::runtime_error(str(
      format("Waypoints have incorrect DOF; expected %d, got %d.")
        % dofs_.size() % waypoints_.cols()));
  }

  std::unordered_set<DegreeOfFreedom const *> unique_dofs;
  for (DegreeOfFreedomPtr const &dof : dofs_) {
    DegreeOfFreedom const *dof_ptr = dof.get();

    if (!dof_ptr) {
      throw std::runtime_error("Path contains NULL DegreeOfFreedom.");
    } else if (unique_dofs.count(dof_ptr)) {
      throw std::runtime_error(str(
        format("Path contains duplicate DOF '%s'.") % dof->getName()));
    }
    unique_dofs.insert(dof_ptr);
  }
}

bool GeometricPath::is_valid() const
{
  for (DegreeOfFreedomPtr const &dof : dofs_) {
    if (!dof) {
      return false;
    }
  }
  return true;
}

size_t GeometricPath::num_dofs() const
{
  return waypoints_.cols();
}

size_t GeometricPath::num_waypoints() const
{
  return waypoints_.rows();
}

double GeometricPath::arc_length() const
{
  double arclength = 0.;

  for (size_t iwaypoint = 1; iwaypoint < waypoints_.size(); ++iwaypoint) {
    VectorXd const &waypoint1 = waypoints_.row(iwaypoint - 1);
    VectorXd const &waypoint2 = waypoints_.row(iwaypoint);
    arclength += (waypoint2 - waypoint1).norm();
  }

  return arclength;
}

std::vector<DegreeOfFreedomPtr> const &GeometricPath::dofs() const
{
  return dofs_;
}

Eigen::MatrixXd &GeometricPath::matrix()
{
  return waypoints_;
}

Eigen::MatrixXd const &GeometricPath::matrix() const
{
  return waypoints_;
}

auto GeometricPath::getWaypoint(size_t i) const -> Waypoint
{
  if (i < num_waypoints()) {
    return Waypoint(this, i);
  } else {
    throw std::runtime_error(str(
      format("Waypoint index is out of range; %d > %d.")
        % i % (num_waypoints() - 1)));
  }
}

