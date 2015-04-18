#include <vector>
#include <boost/make_shared.hpp>
#include <dart/collision/collision.h>
#include <dart/constraint/constraint.h>
#include <dart/dynamics/dynamics.h>
#include <dart/simulation/simulation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <r3/ompl/DARTGeometricStateSpace.h>
#include <r3/ompl/DARTGeometricStateValidityChecker.h>

using ::dart::dynamics::DegreeOfFreedom;
using ::dart::dynamics::Skeleton;
using ::dart::simulation::World;

static void SetDOFValues(std::vector<DegreeOfFreedom *> const &dofs,
                  std::vector<double> const &values)
{
    BOOST_ASSERT(dofs.size() == values.size());

    for (size_t idof = 0; idof < dofs.size(); ++idof) {
        dofs[idof]->setPosition(values[idof]);
    }
}

void OMPLPlan(World *world, Skeleton *skeleton)
{
    using ::boost::dynamic_pointer_cast;
    using ::boost::make_shared;
    using ::dart::collision::CollisionDetector;
    using ::ompl::base::ScopedState;
    using ::ompl::base::SpaceInformation;
    using ::ompl::base::StateValidityChecker;
    using ::ompl::geometric::SimpleSetup;
    using ::r3::ompl::DARTGeometricStateSpace;
    using ::r3::ompl::DARTGeometricStateValidityChecker;

    typedef DARTGeometricStateSpace::StateType DARTState;

    // Setup.
    std::vector<DegreeOfFreedom *> dofs {
        skeleton->getDof("j1"), skeleton->getDof("j2"), skeleton->getDof("j3"),
        skeleton->getDof("j4"), skeleton->getDof("j5"), skeleton->getDof("j6")
    };
    std::vector<double> const weights {
        1., 1., 1., 1., 1., 1. };
    std::vector<double> const q_start {
        1.486,  -1.570,  0.000,  2.034,  4.818,  1.934 };
    std::vector<double> const q_goal {
        1.569,   3.664,  5.501,  1.487,  6.107,  1.400 };

    // Test.
    CollisionDetector *collision_detector
        = world->getConstraintSolver()->getCollisionDetector();
    auto const state_space = make_shared<DARTGeometricStateSpace>(
            dofs, weights, collision_detector);
    auto const space_info = make_shared<SpaceInformation>(state_space);
    auto const validity_checker
        = make_shared<DARTGeometricStateValidityChecker>(space_info);

    ScopedState<DARTGeometricStateSpace> state_start(space_info);
    SetDOFValues(dofs, q_start);
    state_space->GetState(state_start.get());
    state_space->enforceBounds(state_start.get());

    ScopedState<DARTGeometricStateSpace> state_goal(space_info);
    SetDOFValues(dofs, q_goal);
    state_space->GetState(state_goal.get());
    state_space->enforceBounds(state_goal.get());

    auto const setup = make_shared<SimpleSetup>(space_info);
    setup->setStartAndGoalStates(state_start, state_goal);
    setup->setStateValidityChecker(
        dynamic_pointer_cast<StateValidityChecker>(validity_checker));

    setup->print();
    setup->solve(5.);
}
