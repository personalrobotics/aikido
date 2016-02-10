#include <vector>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>
#include <dart/collision/collision.h>
#include <dart/constraint/constraint.h>
#include <dart/simulation/simulation.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <aikido/ompl/plan.hpp>
#include <aikido/ompl/DARTGeometricStateSpace.hpp>
#include <aikido/ompl/DARTGeometricStateValidityChecker.hpp>

using ::dart::collision::CollisionDetector;
using ::dart::dynamics::DegreeOfFreedomPtr;
using ::dart::dynamics::Skeleton;
using ::dart::simulation::World;

typedef ::std::shared_ptr<CollisionDetector> CollisionDetectorPtr;

Eigen::MatrixXd aikido::ompl::Plan(
    CollisionDetectorPtr const &collision_detector,
    std::vector<DegreeOfFreedomPtr> const &dofs,
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
    using ::aikido::ompl::DARTGeometricStateSpace;
    using ::aikido::ompl::DARTGeometricStateValidityChecker;

    size_t const num_dofs = dofs.size();
    if (!collision_detector) {
        throw std::runtime_error("Collision detector is required.");
    }
    if (dof_weights.size() != num_dofs) {
        throw std::runtime_error(str(
            format("Weights have incorrect DOF; %d != %d.")
                % dof_weights.size() % num_dofs));
    }
    if (dof_resolutions.size() != num_dofs) {
        throw std::runtime_error(str(
            format("Resolutions have incorrect DOF; %d != %d.")
                % dof_resolutions.size() % num_dofs));
    }
    if (start_configs.rows() == 0) {
        throw std::runtime_error(
            "At least one start configuration must be specified.");
    }
    if (start_configs.cols() != num_dofs) {
        throw std::runtime_error(str(
            format("Start configurations have incorrect DOF; %d != %d.")
                % start_configs.cols() % num_dofs));
    }
    if (goal_config.size() != num_dofs) {
        throw std::runtime_error(str(
            format("Goal configuration has incorrect DOF; %d != %d.")
                % start_configs.size() % num_dofs));
    }

    // Wrap the DOFs in an OMPL state space.
    auto const state_space = make_shared<DARTGeometricStateSpace>(
            dofs, dof_weights, dof_resolutions, collision_detector);
    auto const space_info = make_shared<SpaceInformation>(state_space);
    auto const setup = make_shared<SimpleSetup>(space_info);

    // Register a StateValidityChecker that checks collision.
    auto const validity_checker
        = make_shared<DARTGeometricStateValidityChecker>(space_info);
    setup->setStateValidityChecker(
        dynamic_pointer_cast<StateValidityChecker>(validity_checker));

    // Add start configurations.
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
    ScopedState<DARTGeometricStateSpace> goal_state(space_info);
    state_space->CreateState(goal_config, goal_state.get());

    if (!state_space->satisfiesBounds(goal_state.get())) {
        throw std::runtime_error("Goal configuration is out of bounds.");
    }

    setup->setGoalState(goal_state);
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
