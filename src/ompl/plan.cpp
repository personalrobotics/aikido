#include <vector>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <dart/collision/collision.h>
#include <dart/constraint/constraint.h>
#include <dart/simulation/simulation.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <r3/ompl/plan.h>
#include <r3/ompl/DARTGeometricStateSpace.h>
#include <r3/ompl/DARTGeometricStateValidityChecker.h>

using ::dart::dynamics::DegreeOfFreedom;
using ::dart::dynamics::Skeleton;
using ::dart::simulation::World;

Eigen::MatrixXd r3::ompl::Plan(
    World *world,
    std::vector<DegreeOfFreedom *> const &dofs,
    Eigen::VectorXd const &dof_weights,
    Eigen::VectorXd const &dof_resolutions,
    Eigen::MatrixXd const &start_configs,
    Eigen::VectorXd const &goal_config)
{
    using ::boost::format;
    using ::boost::str;
    using ::boost::dynamic_pointer_cast;
    using ::boost::make_shared;
    using ::dart::collision::CollisionDetector;
    using ::ompl::base::State;
    using ::ompl::base::ScopedState;
    using ::ompl::base::SpaceInformation;
    using ::ompl::base::StateValidityChecker;
    using ::ompl::control::PathControl;
    using ::ompl::geometric::PathGeometric;
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

        if (!state_space->satisfiesBounds(start_state.get())) {
            throw std::runtime_error(str(
                format("Start configuration %d is out of bounds.") % iconfig));
        }

        setup->addStartState(start_state);
    }

    // Add the goal configuration.
    // TODO: Why doesn't OMPL support multiple goal configurations?
    std::cout << "Adding goal configuration" << std::endl;
    ScopedState<DARTGeometricStateSpace> goal_state(space_info);
    state_space->CreateState(goal_config, goal_state.get());

    if (!state_space->satisfiesBounds(goal_state.get())) {
        throw std::runtime_error("Goal configuration is out of bounds.");
    }

    setup->setGoalState(goal_state);

    setup->print();
    setup->solve(5.);

    // Convert the output path to an Eigen matrix.
    if (setup->haveExactSolutionPath()) {
        PathGeometric const geometric_path = setup->getSolutionPath();

        size_t const num_waypoints = geometric_path.getStateCount();
        Eigen::MatrixXd matrix_path(num_waypoints, dofs.size());

        for (size_t iwaypoint = 0; iwaypoint < num_waypoints; ++iwaypoint) {
            State const *const waypoint = geometric_path.getState(iwaypoint);
            matrix_path.row(iwaypoint) = state_space->ExtractState(
                waypoint->as<DARTGeometricStateSpace::StateType>()
            );
        }
        return matrix_path;
    } else {
        throw std::runtime_error("Planning failed.");
    }
}
