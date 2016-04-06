#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <muul/util/logging.h>
#include <dart/optimizer/nlopt/NloptSolver.h>
#include <dart/common/Console.h>
#include <dart/dynamics/DegreeOfFreedom.h>
#include <dart/dynamics/MetaSkeleton.h>
#include <dart/collision/CollisionDetector.h>
#include <muul/projection/VectorFieldPlanner.h>

namespace muul {
namespace projection {

static void CheckDofLimits(
  dart::dynamics::MetaSkeletonPtr const &skeleton,
  Eigen::VectorXd const &q, Eigen::VectorXd const &qd)
{
  using boost::format;
  using boost::str;
  using dart::dynamics::DegreeOfFreedom;

  for (size_t i = 0; i < skeleton->getNumDofs(); ++i) {
    DegreeOfFreedom *const dof = skeleton->getDof(i);

    if (q[i] < dof->getPositionLowerLimit()) {
      throw DofLimitError(dof, str(
        format("DOF '%s' exceeds lower position limit: %f < %f")
        % dof->getName() % q[i] % dof->getPositionLowerLimit()));
    } else if (q[i] > dof->getPositionUpperLimit()) {
      throw DofLimitError(dof, str(
        format("DOF '%s' exceeds upper position limit: %f > %f")
        % dof->getName() % q[i] % dof->getPositionUpperLimit()));
    } else if (qd[i] < dof->getVelocityLowerLimit()) {
      throw DofLimitError(dof, str(
        format("DOF '%s' exceeds lower velocity limit: %f < %f")
        % dof->getName() % qd[i] % dof->getVelocityLowerLimit()));
    } else if (qd[i] > dof->getVelocityUpperLimit()) {
      throw DofLimitError(dof, str(
        format("DOF '%s' exceeds upper velocity limit: %f > %f")
        % dof->getName() % qd[i] % dof->getVelocityUpperLimit()));
    }
  }
}

static void CheckCollision(
  dart::dynamics::MetaSkeletonPtr const &skeleton)
{
}

aikido::path::TrajectoryPtr Plan(
  dart::dynamics::MetaSkeletonPtr const &skeleton, double dt,
  VectorFieldCallback const &vector_field_cb, StatusCallback const &status_cb)
{
  using boost::format;
  using boost::str;
  using dart::dynamics::DegreeOfFreedom;
  using dart::dynamics::DegreeOfFreedomPtr;
  using aikido::path::CubicSplineTrajectory;
  using CubicSplineProblem = CubicSplineTrajectory::SplineProblem;

  struct Knot {
    double t;
    Eigen::Matrix<double, 2, Eigen::Dynamic> values;
  };

  size_t const num_dof = skeleton->getNumDofs();

  std::vector<Knot> knots;
  Status::Enum termination_status = Status::CONTINUE;
  std::exception_ptr termination_error;

  ptrdiff_t cache_index = -1;
  ptrdiff_t index = 0;
  double t = 0;
  Eigen::VectorXd q = skeleton->getPositions();
  Eigen::VectorXd qd(num_dof);
  assert(q.size() == num_dof);


  do {
    // Evaluate the vector field.
    try {
      if (!vector_field_cb(skeleton, t, &qd)) {
        throw VectorFieldTerminated("Failed evaluating VectorField.");
      }
    } catch (VectorFieldTerminated const &e) {
      termination_status = Status::TERMINATE;
      termination_error = std::current_exception();
      break;
    }

    if (qd.size() != num_dof) {
      throw std::length_error(
        "Vector field returned an incorrect number of DOF velocities.");
    }

    // TODO: This should be computed from the DOF resolutions.
    size_t const num_steps = 1;

    // Compute the number of collision checks we need.
    for (int istep = 0; istep < num_steps; ++istep) {
      try {
        CheckDofLimits(skeleton, q, qd);
        skeleton->setPositions(q);
        CheckCollision(skeleton);
      } catch (VectorFieldTerminated const &e) {
        termination_status = Status::TERMINATE;
        termination_error = std::current_exception();
        break;
      }
      
      // Insert the waypoint.
      assert(q.size() == num_dof);
      assert(qd.size() == num_dof);

      Knot knot;
      knot.t = t;
      knot.values.resize(2, num_dof);
      knot.values.row(0) = q;
      knot.values.row(1) = qd;
      knots.push_back(knot);

      // Take a step.
      q += dt * qd;
      t += dt;
      index++;
      
      // Check if we should terminate.
      termination_status = status_cb(skeleton, t);
      if (termination_status == Status::CACHE_AND_CONTINUE
       || termination_status == Status::CACHE_AND_TERMINATE) {
        cache_index = index;
      }
    }
  } while (termination_status != Status::TERMINATE
        && termination_status != Status::CACHE_AND_TERMINATE);

  if (cache_index >= 0) {
    // Print the termination condition.
    if (termination_error) {
      try {
        std::rethrow_exception(termination_error);
      } catch (VectorFieldTerminated const &e) {
        dtwarn << "[VectorFieldPlanner::Plan] Terminated early: "
               << e.what() << '\n';
      }
    }

    // Construct the output spline.
    Eigen::VectorXd times(cache_index);
    std::transform(knots.begin(), knots.begin() + cache_index, times.data(),
                   [](Knot const &knot) { return knot.t; });

    CubicSplineProblem problem(times, 4, num_dof);
    for (size_t iknot = 0; iknot < cache_index; ++iknot) {
      problem.addConstantConstraint(iknot, 0, knots[iknot].values.row(0));

      if (iknot != 0 && iknot != cache_index - 1) {
        problem.addContinuityConstraint(iknot, 1);
        problem.addContinuityConstraint(iknot, 2);
      }
    }

    problem.addConstantConstraint(0, 1, knots.front().values.row(1));
    problem.addConstantConstraint(cache_index - 1, 1,
      knots[cache_index - 1].values.row(1));

    return boost::make_shared<aikido::path::CubicSplineTrajectory>(problem.fit());
  }
  // Re-throw whatever error caused planning to fail.
  else if (termination_error) {
    std::rethrow_exception(termination_error);
  } else {
    throw VectorFieldTerminated(
      "Planning was terminated by the StatusCallback.");
  }
}

} // namespace projection
} // namespace muul
