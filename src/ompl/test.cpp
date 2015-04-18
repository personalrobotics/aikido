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

void OMPLPlan(
    World *world,
    std::vector<DegreeOfFreedom *> const &dofs,
    Eigen::VectorXd const &dof_weights,
    Eigen::VectorXd const &dof_resolutions,
    Eigen::MatrixXd const &start_configs,
    Eigen::MatrixXd const &goal_configs)
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

    // Wrap the DOFs in an OMPL state space.
    CollisionDetector *collision_detector
        = world->getConstraintSolver()->getCollisionDetector();
    auto const state_space = make_shared<DARTGeometricStateSpace>(
            dofs, dof_weights, dof_resolutions, collision_detector);
    auto const space_info = make_shared<SpaceInformation>(state_space);
    auto const setup = make_shared<SimpleSetup>(space_info);

    // Add start configurations.
    for (size_t iconfig = 0; iconfig < start_configs.rows(); ++iconfig) {
        ScopedState<DARTGeometricStateSpace> state(space_info);

        SetDOFValues(dofs, start_configs.row(iconfig));
        state_space->GetState(state.get());

        setup->addStartState(state);
    }

#if 0
    // Add goal configurations.
    for (size_t iconfig = 0; iconfig < start_configs; ++iconfig) {
        ScopedState<DARTGeometricStateSpace> state(space_info);

        SetDOFValues(dofs, goal_configs.row(iconfig));
        state_space->GetState(state_start.get());

        addStartState(state)
    }
#endif

    // Register a StateValidityChecker that checks collision.
    auto const validity_checker
        = make_shared<DARTGeometricStateValidityChecker>(space_info);
    setup->setStateValidityChecker(
        dynamic_pointer_cast<StateValidityChecker>(validity_checker));

    setup->print();
    setup->solve(5.);
}

void OMPLPlan(World *world, Skeleton *skeleton)
{
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

}
