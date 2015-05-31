#include <list>
#include <unordered_set>
#include <boost/format.hpp>
#include <unsupported/Eigen/Splines>
#include <r3/path/SplineTrajectory.h>

using dart::dynamics::DegreeOfFreedomPtr;
using r3::path::SplineTrajectory;

SplineTrajectory::SplineTrajectory(
    std::vector<DegreeOfFreedomPtr> const &dofs,
    std::vector<Knot> const &knots, size_t order)
  : order_(order)
  , dofs_(dofs)
  , knots_(knots)
{
  using boost::format;
  using boost::str;
  using dart::dynamics::DegreeOfFreedom;

  // Validate the DOFs.
  std::unordered_set<DegreeOfFreedom const *> used_dofs;
  for (DegreeOfFreedomPtr const &dof : dofs) {
    DegreeOfFreedom const *dof_ptr = dof.get();

    if (!dof_ptr) {
      throw std::runtime_error("DegreeOfFreedom is NULL.");
    } else if (used_dofs.count(dof_ptr)) {
      throw std::runtime_error(str(
        format("Duplicate DOF '%s'.") % dof_ptr->getName()));
    }
  }

  // Validate the knots.
  double t_prev = -std::numeric_limits<double>::max();

  for (size_t iknot = 0; iknot < knots.size(); ++iknot) {
    Knot const &knot = knots[iknot];

    if (knot.t <= t_prev) {
      throw std::runtime_error(str(
        format("Time is not monotone: Knot %d has time %f <= %f.")
          % (iknot + 1) % knot.t % t_prev));
    } else if (knot.values.cols() != dofs.size()) {
      throw std::runtime_error(str(
        format("Knot %d has incorrect DOF: got %d, expected %d.")
          % (iknot + 1) % knot.values.cols() % dofs.size()));
    } else if (knot.values.rows() != order) {
      throw std::runtime_error(str(
        format("Knot %d has incorrect order; got %d, expected %d.")
          % (iknot + 1) % knot.values.rows() % order_));
    }

    t_prev = knot.t;
  }
}

SplineTrajectory::~SplineTrajectory()
{
}

double SplineTrajectory::start_time() const
{
  if (!knots_.empty()) {
    return knots_.front().t;
  } else {
    return 0;
  }
}

double SplineTrajectory::end_time() const
{
  if (!knots_.empty()) {
    return knots_.back().t;
  } else {
    return 0;
  }
}

size_t const SplineTrajectory::num_dof() const
{
  return dofs_.size();
}

size_t SplineTrajectory::order() const
{
  return order_;
}

double SplineTrajectory::duration() const
{
  return end_time() - start_time();
}

std::string const &SplineTrajectory::type() const
{
  return static_type();
}

std::string const &SplineTrajectory::static_type()
{
  static std::string const type = "SplineTrajectory";
  return type;
}

double SplineTrajectory::sample(double t, size_t order) const
{
  return 0;
}

Eigen::MatrixXd SplineTrajectory::computeExponents(double t, size_t num_coeffs)
{
  Eigen::MatrixXd t_exponents(num_coeffs, num_coeffs);

  for (size_t i = 0; i < num_coeffs; ++i) {
    for (size_t j = 0; j < num_coeffs; ++j) {
      if (j > i) {
        t_exponents(i, j) = std::pow(t, j - i);
      } else if (j == i) {
        t_exponents(i, j) = 1;
      } else {
        t_exponents(i, j) = 0;
      }
    }
  }
  return t_exponents;
}

Eigen::MatrixXd SplineTrajectory::computeDerivativeMatrix(size_t num_coeffs)
{
  Eigen::MatrixXd coefficients(num_coeffs, num_coeffs);
  coefficients.setZero();
  coefficients.row(0).setOnes();

  for (size_t i = 1; i < num_coeffs; ++i) {
    for (size_t j = i; j < num_coeffs; ++j) {
      coefficients(i, j) = (j - i + 1) * coefficients(i - 1, j);
    }
  }
  return coefficients;
}

auto SplineTrajectory::computeCoefficients(
    std::vector<Knot> const &knots, size_t degree, size_t num_dofs) -> SplineFit
{
  using boost::format;
  using boost::str;

  size_t const num_knots = knots.size();
  size_t const num_splines = num_knots - 1;
  size_t const num_coeffs = degree + 1;
  size_t const num_derivatives = num_coeffs / 2;
  size_t const dim = num_splines * num_coeffs;

  Eigen::MatrixXd const coefficients = computeDerivativeMatrix(num_coeffs);

  SplineFit splineFit;
  splineFit.A.setZero(dim, dim);
  splineFit.b.setZero(dim, num_dofs);

  size_t irow = 0;
  size_t icol = 0;

  for (size_t iknot = 0; iknot < num_knots; ++iknot) {
    Knot const &knot = knots[iknot];

    if (knot.values.rows() != num_derivatives) {
      throw std::runtime_error(str(
        format("Knot %d has incorrect number of derivatives:"
               " expected %d, got %d.")
          % iknot % num_derivatives % knot.values.rows()));
    }

    Eigen::MatrixXd const t_exponents
      = computeExponents(knot.t, num_coeffs);
    Eigen::MatrixXd const A_block
      = coefficients.cwiseProduct(t_exponents)
        .block(0, 0, num_derivatives, num_coeffs);

    // For the spline that ends at knot.
    if (iknot > 0) {
      splineFit.A.block(irow, icol, num_derivatives, num_coeffs) = A_block;
      splineFit.b.block(irow, 0, num_derivatives, num_dofs) = knot.values;
      irow += num_derivatives;
      icol += num_coeffs;
    }

    // For the spline that starts at knot.
    if (iknot + 1 < num_knots) {
      splineFit.A.block(irow, icol, num_derivatives, num_coeffs) = A_block;
      splineFit.b.block(irow, 0, num_derivatives, num_dofs) = knot.values;
      irow += num_derivatives;
    }
  }

  // Build the output matrices.
  return splineFit;
}
