#ifndef SPLINETRAJECTORY_H_
#define SPLINETRAJECTORY_H_
#include <memory>
#include <Eigen/Dense>
#include <dart/dynamics/dynamics.h>
#include <r3/path/Trajectory.h>

namespace r3 {
namespace path {


class Spline {
public:
  struct Knot {
    double t;
    Eigen::MatrixXd values;
  };

  Spline(
    std::vector<double> const &times,
    Eigen::MatrixXd const &coefficients);

  size_t order() const;
  double start_time() const;
  double end_time() const;

  double interpolate(double t, size_t order = 0) const;

  static std::vector<Spline> fit(std::vector<Knot> const &knots);

  // TODO: Make this stuff private.
  struct Problem {
    Eigen::MatrixXd A;
    Eigen::MatrixXd b;
    std::vector<double> times;
  };

  static Eigen::MatrixXd computeExponents(double t, size_t num_coeffs);
  static Eigen::MatrixXd computeDerivativeMatrix(size_t num_coeffs);
  static Problem createProblem(std::vector<Knot> const &knots);
  static std::vector<Spline> solveProblem(Problem const &problem);

  size_t getSplineIndex(double t) const;

private:
  std::vector<double> times_;
  Eigen::MatrixXd coefficients_;
};


class SplineTrajectory : public Trajectory {
public:
  SplineTrajectory(
    std::vector<dart::dynamics::DegreeOfFreedomPtr> const &dofs,
    std::vector<Spline> const &splines);

  virtual ~SplineTrajectory();

  virtual size_t const num_dof() const;
  virtual size_t order() const;
  virtual double start_time() const;
  virtual double end_time() const;
  virtual double duration() const;

  virtual std::string const &type() const;
  static std::string const &static_type();

  virtual Eigen::VectorXd sample(double t, size_t order) const;

private:
  std::vector<dart::dynamics::DegreeOfFreedomPtr> dofs_;
  std::vector<Spline> splines_;
};

typedef std::shared_ptr<SplineTrajectory> SplineTrajectoryPtr;
typedef std::shared_ptr<SplineTrajectory const> SplineTrajectoryConstPtr;

}
}

#endif
