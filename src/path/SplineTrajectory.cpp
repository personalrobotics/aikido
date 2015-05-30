#include <unordered_set>
#include <boost/format.hpp>
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
  return "SplineTrajectory";
}

double SplineTrajectory::sample(double t, size_t order) const
{
}
