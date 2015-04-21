#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <dart/collision/collision.h>
#include <dart/dynamics/dynamics.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <r3/ompl/DARTGeometricStateSpace.h>

using ::dart::collision::CollisionDetector;
using ::dart::dynamics::DegreeOfFreedom;
using ::dart::dynamics::Skeleton;
using ::r3::ompl::DARTGeometricStateSpace;

typedef ::ompl::base::SO2StateSpace::StateType SO2State;
typedef ::ompl::base::RealVectorStateSpace::StateType RealVectorState;

DARTGeometricStateSpace::DARTGeometricStateSpace(
        std::vector<DegreeOfFreedom *> const &dofs,
        Eigen::VectorXd const &weights,
        Eigen::VectorXd const &resolutions,
        CollisionDetector *collision_detector)
    : dofs_(dofs)
    , is_circular_(dofs.size())
    , collision_detector_(collision_detector)
{
    using ::boost::make_shared;
    using ::ompl::base::RealVectorStateSpace;
    using ::ompl::base::SO2StateSpace;
    using ::ompl::base::StateSpacePtr;

    BOOST_ASSERT(dofs.size() == weights.size());
    BOOST_ASSERT(dofs.size() == resolutions.size());
    BOOST_ASSERT(collision_detector);

    for (size_t idof = 0; idof < dofs.size(); ++idof) {
        DegreeOfFreedom *dof = dofs[idof];

        // Infer whether the joint is circular or not. Circular joints are
        // mapped to SO2 state spaces, instead of real-valued state spaces.
        is_circular_[idof] = IsDOFCircular(dof);

        // Construct a one-dimensional state space for each DOF. Add them, in
        // the same order as "dofs", to this CompoundStateSpace.
        StateSpacePtr state_space;

        if (is_circular_[idof]) {
            state_space = make_shared<SO2StateSpace>();
        } else {
            auto const state_space_impl = make_shared<RealVectorStateSpace>(1);
            state_space_impl->setBounds(
                dof->getPositionLowerLimit(),
                dof->getPositionUpperLimit()
            );
            state_space = state_space_impl;
        }

        // OMPL specifies the collision checking resolution as a ratio of the
        // maximum extent. We do the necessary conversion here.
        state_space->setLongestValidSegmentFraction(
            resolutions[idof] / state_space->getMaximumExtent()
        );

        state_space->setName(dof->getName());
        addSubspace(state_space, weights[idof]);

        skeletons_.insert(dof->getSkeleton());
    }
}

void DARTGeometricStateSpace::SetState(StateType const *state)
{
    Eigen::VectorXd const values = ExtractState(state);

    // Update joint values.
    for (size_t idof = 0; idof < dofs_.size(); ++idof) {
        dofs_[idof]->setPosition(values[idof]);
    }

    // Compute forward kinematics.
    for (Skeleton *skeleton : skeletons_) {
        skeleton->computeForwardKinematics(true, false, false);
    }
}

void DARTGeometricStateSpace::GetState(StateType *state) const
{
    Eigen::VectorXd dof_values(dofs_.size());

    for (size_t idof = 0; idof < dofs_.size(); ++idof) {
        dof_values[idof] = dofs_[idof]->getPosition();
    }

    CreateState(dof_values, state);
}

Eigen::VectorXd DARTGeometricStateSpace::ExtractState(StateType const *state) const
{
    Eigen::VectorXd dof_values(dofs_.size());

    for (size_t idof = 0; idof < dofs_.size(); ++idof) {
        double &value = dof_values[idof];

        if (is_circular_[idof]) {

            value = (*state)[idof]->as<SO2State>()->value;
        } else {
            value = (*state)[idof]->as<RealVectorState>()->values[0];
        }
    }

    return dof_values;
}

void DARTGeometricStateSpace::CreateState(Eigen::VectorXd const &dof_values,
                                          StateType *state) const
{
    BOOST_ASSERT(dofs_.size() == dof_values.size());

    for (size_t idof = 0; idof < dofs_.size(); ++idof) {
        double const &position = dof_values[idof];

        if (is_circular_[idof]) {
            auto *so2_state = (*state)[idof]->as<SO2State>();
            so2_state->value = position;
            getSubspace(idof)->enforceBounds(so2_state);
        } else {
            (*state)[idof]->as<RealVectorState>()->values[0] = position;
        }
    }
}

bool DARTGeometricStateSpace::IsInCollision()
{
    // TODO: We should only collision check the BodyNodes that are related to
    // this state space.
    return collision_detector_->detectCollision(false, false);
}

bool DARTGeometricStateSpace::IsDOFCircular(DegreeOfFreedom const *dof)
{
    bool const has_lower = ::std::isfinite(dof->getPositionLowerLimit());
    bool const has_upper = ::std::isfinite(dof->getPositionUpperLimit());

    // TODO: We also need to check whether this DOF is revolute.

    return !has_lower && !has_upper;
}

