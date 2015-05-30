#ifndef GEOMETRICPATH_H_
#define GEOMETRICPATH_H_
#include <Eigen/Dense>
#include <dart/dynamics/dynamics.h>

namespace r3 {
namespace path {

class GeometricPath {
public:
  class Waypoint {
  public:
    std::vector<dart::dynamics::DegreeOfFreedomPtr> const &dofs() const;
    Eigen::VectorXd positions() const;

  private:
    GeometricPath const *path_;
    size_t index_;

    Waypoint(GeometricPath const *path, size_t index);

    friend class GeometricPath;
  };

  GeometricPath(std::vector<dart::dynamics::DegreeOfFreedomPtr> const &dofs,
                Eigen::MatrixXd const &waypoints);

  size_t num_dofs() const;
  size_t num_waypoints() const;

  bool is_valid() const;

  double arc_length() const;

  std::vector<dart::dynamics::DegreeOfFreedomPtr> const &dofs() const;

  Eigen::MatrixXd &matrix();
  Eigen::MatrixXd const &matrix() const;

  Waypoint getWaypoint(size_t i) const;

private:
  std::vector<dart::dynamics::DegreeOfFreedomPtr> dofs_;
  Eigen::MatrixXd waypoints_;

  friend class Waypoint;
};

}
}

#endif
