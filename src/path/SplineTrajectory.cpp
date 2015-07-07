#include <list>
#include <unordered_set>
#include <boost/format.hpp>
#include <unsupported/Eigen/Splines>
#include <r3/path/SplineTrajectory.h>

using dart::dynamics::DegreeOfFreedomPtr;
using r3::path::Spline;
using r3::path::SplineTrajectory;

/*
 * Spline
 */
Spline::Spline(
      std::vector<double> const &times,
      Eigen::MatrixXd const &coefficients)
  : times_(times)
  , coefficients_(coefficients)
{
  BOOST_ASSERT(std::is_sorted(std::begin(times), std::end(times)));
  BOOST_ASSERT(times_.size() == coefficients.rows() + 1);
}

size_t Spline::order() const
{
  return coefficients_.cols() - 1;
}

double Spline::start_time() const
{
  return times_.front();
}

double Spline::end_time() const
{
  return times_.back();
}

double Spline::interpolate(double t, size_t order) const
{
  if (order != 0) {
    throw std::runtime_error("Derivatives are not yet implemented.");
  }

  size_t const numCoeffs = coefficients_.cols();
  size_t const splineIndex = getSplineIndex(t);

  Eigen::VectorXd tExponents(numCoeffs);
  tExponents[0] = 1;

  for (size_t i = 1; i < numCoeffs; ++i) {
    tExponents[i] = tExponents[i - 1] * t; 
  }

  return coefficients_.row(splineIndex).dot(tExponents);
}

std::vector<Spline> Spline::fit(std::vector<Knot> const &knots)
{
  bool const is_monotone = std::is_sorted(std::begin(knots), std::end(knots),
    [](Knot const &x, Knot const &y) {
      return x.t < y.t;
    }
  );

  if (!is_monotone) {
    throw std::runtime_error("Knot times are not monotonic.");
  }

  return solveProblem(createProblem(knots));
}

size_t Spline::getSplineIndex(double t) const
{
  if (t <= times_.front()) {
    return 0;
  } else if (t >= times_.back()) {
    return coefficients_.rows() - 1;
  } else {
    auto it = std::lower_bound(std::begin(times_), std::end(times_), t);
    return it - std::begin(times_) - 1;
  }
}

Eigen::MatrixXd Spline::computeExponents(double t, size_t num_coeffs)
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

Eigen::MatrixXd Spline::computeDerivativeMatrix(size_t num_coeffs)
{
  Eigen::MatrixXd coefficients(num_coeffs, num_coeffs);
  coefficients.setZero();

  if (num_coeffs > 0) {
    coefficients.row(0).setOnes();
  }

  for (size_t i = 1; i < num_coeffs; ++i) {
    for (size_t j = i; j < num_coeffs; ++j) {
      coefficients(i, j) = (j - i + 1) * coefficients(i - 1, j);
    }
  }
  return coefficients;
}

auto Spline::createProblem(std::vector<Knot> const &knots) -> Problem
{
  using boost::format;
  using boost::str;

  if (knots.size() < 2) {
    throw std::runtime_error(str(
      format("Spline requires at least two knots; got %d.") % knots.size()));
  }

  size_t const num_derivatives = knots.front().values.rows();
  size_t const num_dofs = knots.front().values.cols();
  size_t const num_coeffs = 2 * num_derivatives;
  size_t const num_knots = knots.size();
  size_t const num_splines = num_knots - 1;
  size_t const dim = num_splines * num_coeffs;
  Eigen::MatrixXd const coefficients = computeDerivativeMatrix(num_coeffs);

  Problem problem;
  problem.A.setZero(dim, dim);
  problem.b.setZero(dim, num_dofs);
  problem.times.resize(num_knots);

  size_t irow = 0;
  size_t icol = 0;

  for (size_t iknot = 0; iknot < num_knots; ++iknot) {
    Knot const &knot = knots[iknot];
    problem.times[iknot] = knot.t;

    if (knot.values.rows() != num_derivatives) {
      throw std::runtime_error(str(
        format("Knot %d has incorrect number of derivatives:"
               " expected %d, got %d.")
          % iknot % num_derivatives % knot.values.rows()));
    } else if (knot.values.cols() != num_dofs) {
      throw std::runtime_error(str(
        format("Knot %d has incorrect number of DOFs:"
               " expected %d, got %d.")
          % iknot % num_dofs % knot.values.cols()));
    }

    Eigen::MatrixXd const t_exponents
      = computeExponents(knot.t, num_coeffs);
    Eigen::MatrixXd const A_block
      = coefficients.cwiseProduct(t_exponents)
        .block(0, 0, num_derivatives, num_coeffs);

    // TODO: We know that problem.A is block-diagonal. Invert it in-place.

    // For the spline that ends at knot.
    if (iknot > 0) {
      problem.A.block(irow, icol, num_derivatives, num_coeffs) = A_block;
      problem.b.block(irow, 0, num_derivatives, num_dofs) = knot.values;
      irow += num_derivatives;
      icol += num_coeffs;
    }

    // For the spline that starts at knot.
    if (iknot + 1 < num_knots) {
      problem.A.block(irow, icol, num_derivatives, num_coeffs) = A_block;
      problem.b.block(irow, 0, num_derivatives, num_dofs) = knot.values;
      irow += num_derivatives;
    }
  }
  return problem;
}

std::vector<Spline> Spline::solveProblem(Problem const &problem)
{
  size_t const num_dofs = problem.b.cols();
  size_t const num_splines = problem.times.size() - 1;
  size_t const num_coeffs = problem.b.rows() / num_splines;

  std::vector<Spline> splines;
  splines.reserve(num_splines);

  // TODO: We know that problem.A is block-diagonal. Invert it in-place.
  auto solver = problem.A.householderQr();

  for (size_t idof = 0; idof < num_dofs; ++idof) {
    Eigen::VectorXd const coeffsVector = solver.solve(problem.b.col(idof));
    Eigen::MatrixXd coeffs(num_splines, num_coeffs);

    for (size_t i = 0; i < num_splines; ++i) {
      coeffs.row(i) = coeffsVector.segment(i * num_coeffs, num_coeffs);
    }

    splines.push_back(Spline(problem.times, coeffs));
  }

  return splines;
}


/*
 * SplineTrajectory
 */
SplineTrajectory::SplineTrajectory(
    std::vector<DegreeOfFreedomPtr> const &dofs,
    std::vector<Spline> const &splines)
  : dofs_(dofs)
  , splines_(splines)
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
}

SplineTrajectory::~SplineTrajectory()
{
}

double SplineTrajectory::start_time() const
{
  auto const it = std::max_element(std::begin(splines_), std::end(splines_),
    [](Spline const &x, Spline const &y) {
      return x.start_time() < y.start_time();
    }
  );
  return it->start_time();
}

double SplineTrajectory::end_time() const
{
  auto const it = std::max_element(std::begin(splines_), std::end(splines_),
    [](Spline const &x, Spline const &y) {
      return x.end_time() < y.end_time();
    }
  );
  return it->end_time();
}

size_t const SplineTrajectory::num_dof() const
{
  return dofs_.size();
}

size_t SplineTrajectory::order() const
{
  auto const it = std::max_element(std::begin(splines_), std::end(splines_),
    [](Spline const &x, Spline const &y) {
      return x.order() < y.order();
    }
  );
  return it->order();
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
  static std::string const type = "Spline";
  return type;
}

Eigen::VectorXd SplineTrajectory::sample(double t, size_t order) const
{
  Eigen::VectorXd x(dofs_.size());

  for (size_t i = 0; i < dofs_.size(); ++i) {
    x[i] = splines_[i].interpolate(t, order);
  }

  return x;
}
