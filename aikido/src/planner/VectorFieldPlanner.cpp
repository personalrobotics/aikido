#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
//#include <muul/util/logging.h>
#include <dart/optimizer/nlopt/NloptSolver.h>
#include <dart/common/Console.h>
#include <dart/dynamics/DegreeOfFreedom.h>
#include <dart/dynamics/MetaSkeleton.h>
#include <dart/collision/CollisionDetector.h>
#include <aikido/constraint/Testable.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/VectorFieldPlanner.hpp>
#include <aikido/statespace/Interpolator.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/util/VanDerCorput.hpp>

namespace aikido {
namespace planner {

// FIXME: Probably dont need skeleton, stateSpace, and constraint
trajectory::InterpolatedPtr planVF(
  const std::shared_ptr<aikido::statespace::StateSpace>& stateSpace,
  const aikido::statespace::StateSpace::State *startState,
  const aikido::statespace::StateSpace::State *goalState,
  const std::shared_ptr<aikido::statespace::Interpolator>& interpolator,
  const std::shared_ptr<aikido::constraint::Testable>& constraint,
  aikido::planner::PlanningResult& planningResult,
  //dart::dynamics::MetaSkeletonPtr const &skeleton, 
  double dt,
  VectorFieldCallback const &vector_field_cb
  //StatusCallback const &status_cb
  )
{
  using boost::format;
  using boost::str;
  using dart::dynamics::DegreeOfFreedom;
  using dart::dynamics::DegreeOfFreedomPtr;
  using aikido::path::CubicSplineTrajectory;
  using CubicSplineProblem = CubicSplineTrajectory::SplineProblem;

/*
  struct Knot {
    double t;
    aikido::statespace::StateSpace::State qvalues;
    Eigen::VectorXd dqvalues;
  };
*/

  //std::unique_ptr<Trajectory> qvalues(new Trajectory());
  auto returnTraj = std::make_shared<trajectory::Interpolated>(
    stateSpace, interpolator);
  std::vector<Eigen::VectorXd> dqvalues;
  std::vector<double> tvalues;
  Status::Enum termination_status = Status::CONTINUE;
  std::exception_ptr termination_error;

  ptrdiff_t cache_index = -1;
  ptrdiff_t index = 0;
  double t = 0;
  
  if (stateSpace != constraint->getStateSpace()) {
    throw std::invalid_argument(
        "StateSpace of constraint not equal to StateSpace of planning space");
  }
  
  
  size_t const num_dof = stateSpace->getDimension();
  auto currentState=stateSpace->createState();
  auto q=stateSpace->createState();
  Eigen::VectorXd dq(num_dof);
  auto qstep=stateSpace->createState();
  
  stateSpace->copyState(startState,q);
  
  // FIXME: Not sure why this doesnt work: 
  // assert(q.getDimension() == num_dof);


  do {
    // Evaluate the vector field.
  
    if (!vector_field_cb(q, t, &dq)) {
      planningResult.message = "Failed evaluating VectorField";
      return returnTraj;
    } 

    if (dq.size() != num_dof) {
      throw std::length_error(
        "Vector field returned an incorrect number of DOF velocities.");
    }

    // TODO: This should be computed from the DOF resolutions.
    size_t const num_steps = 1;

    // Compute the number of collision checks we need.
    for (int istep = 0; istep < num_steps; ++istep) {

      stateSpace->copyState(currentState,q);
      if(!constraint->isSatisfied(currentState)) {          
        planningResult.message = "Collision detected";
        return returnTraj;
      } 
      
      // Insert the waypoint.
      assert(dq.size() == num_dof);
      
      returnTraj->addWaypoint(istep,q);
      dqvalues.push_back(dq);
      tvalues.push_back(t);

      // Take a step.
      stateSpace->expMap(dt*dq,qstep);
      stateSpace->compose(q,qstep,q);
      t += dt;
      index++;
      
      // Check if we should terminate.
      /*termination_status = status_cb(skeleton, t);
      if (termination_status == Status::CACHE_AND_CONTINUE
       || termination_status == Status::CACHE_AND_TERMINATE) {
        cache_index = index;
      }*/
    }
  } while (termination_status != Status::TERMINATE
        && termination_status != Status::CACHE_AND_TERMINATE);

/*
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
      problem.addConstantConstraint(iknot, 0, knots[iknot].qvalues);

      if (iknot != 0 && iknot != cache_index - 1) {
        problem.addContinuityConstraint(iknot, 1);
        problem.addContinuityConstraint(iknot, 2);
      }
    }

    problem.addConstantConstraint(0, 1, knots.front().dqvalues);
    problem.addConstantConstraint(cache_index - 1, 1,
      knots[cache_index - 1].dqvalues);

    return boost::make_shared<aikido::path::CubicSplineTrajectory>(problem.fit());
    */
    
    return returnTraj;
/*  }
  // Re-throw whatever error caused planner to fail.
  else if (termination_error) {
    std::rethrow_exception(termination_error);
  } else {
    throw VectorFieldTerminated(
      "Planning was terminated by the StatusCallback.");
  }
  */
}

} // namespace planner
} // namespace aikido
