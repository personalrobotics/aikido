#ifndef SPLINETRAJECTORY_H_
#define SPLINETRAJECTORY_H_
#include <Eigen/Dense>
#include <dart/dynamics/dynamics.h>
#include <r3/path/Trajectory.h>

namespace r3 {
namespace path {


class Spline {
public:
  Spline(
    std::vector<double> const &times,
    Eigen::MatrixXd const &coefficients);

  double interpolate(double t, size_t order = 0) const;

private:
  std::vector<double> times_;
  Eigen::MatrixXd coefficients_;

  size_t getSplineIndex(double t) const;
};


class SplineTrajectory : public Trajectory {
public:
  struct Knot {
    double t;
    Eigen::MatrixXd values;
  };

  struct SplineProblem {
    Eigen::MatrixXd A;
    Eigen::MatrixXd b;
    std::vector<double> times;
  };

  SplineTrajectory(
    std::vector<dart::dynamics::DegreeOfFreedomPtr> const &dofs,
    std::vector<Knot> const &knots, size_t order);

  virtual ~SplineTrajectory();

  virtual size_t const num_dof() const;
  virtual size_t order() const;
  virtual double start_time() const;
  virtual double end_time() const;
  virtual double duration() const;

  virtual std::string const &type() const;
  static std::string const &static_type();

  virtual double sample(double t, size_t order) const;

  static Eigen::MatrixXd computeExponents(double t, size_t num_coeffs);
  static Eigen::MatrixXd computeDerivativeMatrix(size_t num_coeffs);
  static SplineProblem createProblem(
    std::vector<Knot> const &knots, size_t degree, size_t num_dofs);
  static std::vector<Spline> solveProblem(SplineProblem const &problem);

private:
  size_t order_;
  std::vector<dart::dynamics::DegreeOfFreedomPtr> dofs_;
  std::vector<Knot> knots_;
};

}
}

#endif
