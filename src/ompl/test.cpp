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
                         Eigen::VectorXd const &values)
{
    BOOST_ASSERT(dofs.size() == values.size());

    for (size_t idof = 0; idof < dofs.size(); ++idof) {
        dofs[idof]->setPosition(values[idof]);
    }
}

void Plan(
    World *world,
    std::vector<DegreeOfFreedom *> const &dofs,
    Eigen::VectorXd const &dof_weights,
    Eigen::VectorXd const &dof_resolutions,
    Eigen::MatrixXd const &start_configs,
    Eigen::VectorXd const &goal_config)
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

    BOOST_ASSERT(world);
    BOOST_ASSERT(dofs.size() == dof_weights.size());
    BOOST_ASSERT(dofs.size() == dof_resolutions.size());
    BOOST_ASSERT(dofs.size() == start_configs.cols());
    BOOST_ASSERT(dofs.size() == goal_config.size());

    // Wrap the DOFs in an OMPL state space.
    std::cout << "Creating StateSpace" << std::endl;

    CollisionDetector *collision_detector
        = world->getConstraintSolver()->getCollisionDetector();
    auto const state_space = make_shared<DARTGeometricStateSpace>(
            dofs, dof_weights, dof_resolutions, collision_detector);
    auto const space_info = make_shared<SpaceInformation>(state_space);
    auto const setup = make_shared<SimpleSetup>(space_info);

    // Register a StateValidityChecker that checks collision.
    std::cout << "Creating StateValidityChecker" << std::endl;
    auto const validity_checker
        = make_shared<DARTGeometricStateValidityChecker>(space_info);
    setup->setStateValidityChecker(
        dynamic_pointer_cast<StateValidityChecker>(validity_checker));

    // Add start configurations.
    std::cout << "Adding start configurations" << std::endl;
    for (size_t iconfig = 0; iconfig < start_configs.rows(); ++iconfig) {
        ScopedState<DARTGeometricStateSpace> start_state(space_info);
        state_space->CreateState(start_configs.row(iconfig), start_state.get());
        setup->addStartState(start_state);
    }

    // Add the goal configuration.
    // TODO: Why doesn't OMPL support multiple goal configurations?
    std::cout << "Adding goal configuration" << std::endl;
    ScopedState<DARTGeometricStateSpace> goal_state(space_info);
    state_space->CreateState(goal_config, goal_state.get());
    setup->setGoalState(goal_state);

    setup->print();
    setup->solve(5.);
}

void OMPLPlan(World *world, Skeleton *skeleton)
{
    BOOST_ASSERT(world);
    BOOST_ASSERT(skeleton);

    std::vector<DegreeOfFreedom *> dofs {
        skeleton->getDof("j1"), skeleton->getDof("j2"), skeleton->getDof("j3"),
        skeleton->getDof("j4"), skeleton->getDof("j5"), skeleton->getDof("j6")
    };

    Eigen::Vector6d weights, resolutions, q_start, q_goal;
    weights << 1., 1., 1., 1., 1., 1.;
    resolutions << 0.02, 0.02, 0.02, 0.02, 0.02, 0.02;
    q_start << 1.486,  -1.570,  0.000,  2.034,  4.818,  1.934;
    q_goal <<  1.569,   3.664,  5.501,  1.487,  6.107,  1.400;

    Plan(world, dofs, weights, resolutions, q_start.transpose(), q_goal);
}
