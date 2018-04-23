#include "aikido/planner/kinodynamics/KinodynamicPlanner.hpp"
#include "aikido/planner/ompl/Planner.hpp"
#include "aikido/planner/kinodynamics/dimt/DoubleIntegratorMinimumTime.h"
#include "aikido/planner/kinodynamics/ompl/DimtStateSpace.h"
#include "aikido/constraint/TestableIntersection.hpp"
#include "aikido/planner/ompl/MotionValidator.hpp"
#include "aikido/planner/ompl/StateValidityChecker.hpp"

namespace aikido {
namespace planner {
namespace kinodynamics {

::ompl::base::SpaceInformationPtr getSpaceInformation(
    DIMTPtr _dimt,
    statespace::StateSpacePtr _stateSpace,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    double _maxDistanceBtwValidityChecks)
{
  // construct the state space we are planning in
  ::ompl::base::StateSpacePtr space = std::make_shared< ::ompl::base::DimtStateSpace >(_dimt);

  ::ompl::base::RealVectorBounds bounds(_dimt->getNumDofs());
  bounds.setLow(-10);
  bounds.setHigh(10);
  space->as<::ompl::base::DimtStateSpace>()->setBounds(bounds);
  ::ompl::base::SpaceInformationPtr si = std::make_shared<::ompl::base::SpaceInformation>(space);

  // Validity checking
  std::vector<constraint::TestablePtr> constraints{
      std::move(_validityConstraint), std::move(_boundsConstraint)};
  auto conjunctionConstraint
      = std::make_shared<aikido::constraint::TestableIntersection>(
          std::move(_stateSpace), std::move(constraints));
  ::ompl::base::StateValidityCheckerPtr vchecker
      = std::make_shared<aikido::planner::ompl::StateValidityChecker>(si, conjunctionConstraint);
  si->setStateValidityChecker(vchecker);

  ::ompl::base::MotionValidatorPtr mvalidator
      = std::make_shared<aikido::planner::ompl::MotionValidator>(si, _maxDistanceBtwValidityChecks);
  si->setMotionValidator(mvalidator);
  si->setup();

  return si;
}

trajectory::InterpolatedPtr planViaConstraint(
    const statespace::StateSpace::State* _start,
    const statespace::StateSpace::State* _goal,
    const statespace::StateSpace::State* _via,
    const Eigen::VectorXd& _viaVelocity,
    dart::dynamics::MetaSkeletonPtr _metaSkeleton,
    statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
    constraint::TestablePtr _validityConstraint,
    constraint::TestablePtr _boundsConstraint,
    double _maxPlanTime,
    double _maxDistanceBtwValidityChecks)
{
  std::size_t numDofs = _metaSkeleton->getNumDofs();
  // Initialize parameters from bounds constraint
  std::vector<double> maxVelocities(numDofs, 0.0);
  std::vector<double> maxAccelerations(numDofs, 0.0);
  for(std::size_t i=0; i<numDofs; i++)
  {
    double absUpperVel = std::abs(_metaSkeleton->getVelocityUpperLimit(i));
    double absLowerVel = std::abs(_metaSkeleton->getVelocityLowerLimit(i));
    maxVelocities[i] = absUpperVel>absLowerVel ? absLowerVel : absUpperVel;

    double absUpperAccl = std::abs(_metaSkeleton->getAccelerationUpperLimit(i));
    double absLowerAccl = std::abs(_metaSkeleton->getAccelerationLowerLimit(i));
    maxAccelerations[i] = absUpperAccl>absLowerAccl ? absLowerAccl : absUpperAccl;
  }
  DIMTPtr dimt = std::make_shared<DIMT>( numDofs, maxAccelerations, maxVelocities );

  auto si = getSpaceInformation(dimt, _metaSkeletonStateSpace,
                                _validityConstraint,
                                _boundsConstraint,
                                _maxDistanceBtwValidityChecks);



  // plan from start to via


  // plan from via to goal

  // concatenate two path





/*

    // Set custom start and goal
    ompl::base::State *start_s = space->allocState();
    ompl::base::State *goal_s = space->allocState();
    for (int i = 0; i < param.dimensions; i++)
    {
        start_s->as<ob::RealVectorStateSpace::StateType>()->values[i] = -5;
        goal_s->as<ob::RealVectorStateSpace::StateType>()->values[i] = 5;
    }
    ob::ScopedState<ompl::base::RealVectorStateSpace> start(space, start_s);
    ob::ScopedState<ompl::base::RealVectorStateSpace> goal(space, goal_s);
    // Set random start and goal
    // ob::ScopedState<> start(space);
    // start.random();
    // ob::ScopedState<> goal(space);
    // goal.random();
    // Setup Problem Definition
    ob::ProblemDefinitionPtr pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);
    // Construct Planner
    // ob::PlannerPtr planner(new ompl::geometric::RRTConnect(si));
    ob::PlannerPtr planner(new og::RRTstar(si));
    // Set the problem instance for our planner to solve
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Run planner
    ob::PlannerStatus solved = planner->solve(5.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // get the goal representation from the problem definition (not the same as
        // the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        // print the path to screen
        path->print(std::cout);
        // Print to File
        // std::ofstream myfile;
        // myfile.open("geometric_pointmass2d.txt");
        // path->printAsMatrix(std::cout);
        // myfile.close();
    }
*/

    return nullptr;
}

} // namespace kinodynamics
} // namespace planner
} // namespace aikido
