#include <aikido/planner/VectorFieldPlanner.hpp>
#include <boost/format.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpaceSaver.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/common/Spline.hpp>
#include "VectorFieldPlannerExceptions.h"

using dart::common::make_unique;
using aikido::statespace::dart::MetaSkeletonStateSpaceSaver;

namespace aikido {
namespace planner {
namespace vectorfield {

static void CheckDofLimits(
  const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
  Eigen::VectorXd const &q, Eigen::VectorXd const &qd)
{
  using boost::format;
  using boost::str;
  using dart::dynamics::DegreeOfFreedom;

  for (size_t i = 0; i < stateSpace->getMetaSkeleton()->getNumDofs(); ++i) {
    DegreeOfFreedom *const dof = stateSpace->getMetaSkeleton()->getDof(i);

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
  const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace)
{
    DART_UNUSED(stateSpace);
}

std::unique_ptr<aikido::trajectory::Spline>planPathByVectorField(
  const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
  double dt,
  const VectorFieldCallback& vectorFiledCb,
  const VectorFieldStatusCallback& statusCb)
{
    using boost::format;
    using boost::str;
    using dart::dynamics::DegreeOfFreedom;
    using dart::dynamics::DegreeOfFreedomPtr;
    using aikido::trajectory::Spline;
    using CubicSplineProblem
        = aikido::common::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

    struct Knot {
      double t;
      Eigen::Matrix<double, 2, Eigen::Dynamic> values;
    };

    auto saver = MetaSkeletonStateSpaceSaver(stateSpace);
    DART_UNUSED(saver);

    size_t const num_dof = stateSpace->getDimension();

    std::vector<Knot> knots;
    VectorFieldPlannerStatus::Enum termination_status = VectorFieldPlannerStatus::CONTINUE;
    std::exception_ptr termination_error;

    ptrdiff_t cache_index = -1;
    ptrdiff_t index = 0;
    double t = 0;
    Eigen::VectorXd q = stateSpace->getMetaSkeleton()->getPositions();
    Eigen::VectorXd qd(num_dof);
    assert( static_cast<size_t>(q.size()) == num_dof);

    do {
      // Evaluate the vector field.
      try {
        if (!vectorFiledCb(stateSpace, t, &qd)) {
          throw VectorFieldTerminated("Failed evaluating VectorField.");
        }
      } catch (VectorFieldTerminated const &e) {
        termination_status = VectorFieldPlannerStatus::TERMINATE;
        termination_error = std::current_exception();
        break;
      }

      if ( static_cast<size_t>( qd.size() ) != num_dof) {
        throw std::length_error(
          "Vector field returned an incorrect number of DOF velocities.");
      }

      // TODO: This should be computed from the DOF resolutions.
      size_t const num_steps = 1;

      // Compute the number of collision checks we need.
      for (size_t istep = 0; istep < num_steps; ++istep) {
        try {
          CheckDofLimits(stateSpace, q, qd);
          stateSpace->getMetaSkeleton()->setPositions(q);
          CheckCollision(stateSpace);
        } catch (VectorFieldTerminated const &e) {
          termination_status = VectorFieldPlannerStatus::TERMINATE;
          termination_error = std::current_exception();
          break;
        }

        // Insert the waypoint.
        assert(static_cast<size_t>(q.size()) == num_dof);
        assert(static_cast<size_t>(qd.size()) == num_dof);

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
        termination_status = statusCb(stateSpace, t);
        if (termination_status == VectorFieldPlannerStatus::CACHE_AND_CONTINUE
         || termination_status == VectorFieldPlannerStatus::CACHE_AND_TERMINATE) {
          cache_index = index;
        }
      }
    } while (termination_status != VectorFieldPlannerStatus::TERMINATE
          && termination_status != VectorFieldPlannerStatus::CACHE_AND_TERMINATE);

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

      auto _outputTrajectory = make_unique<aikido::trajectory::Spline>(stateSpace);
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

      const auto spline = problem.fit();
      return _outputTrajectory;
    }
    // Re-throw whatever error caused planning to fail.
    else if (termination_error) {
      std::rethrow_exception(termination_error);
    } else {
      throw VectorFieldTerminated(
        "Planning was terminated by the StatusCallback.");
    }

}

std::unique_ptr<aikido::trajectory::Spline> planStrightLine(
  const aikido::statespace::dart::MetaSkeletonStateSpacePtr& stateSpace,
  const Eigen::VectorXd& startPosition,
  const Eigen::VectorXd& goalPosition)
{

}

} // namespace vectorfield
} // namespace planner
} // namespace aikido


