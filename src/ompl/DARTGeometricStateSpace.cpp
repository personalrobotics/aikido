#include <boost/make_shared.hpp>
#include <dart/collision/collision.h>
#include <dart/dynamics/dynamics.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <r3/ompl/DARTGeometricStateSpace.h>

using ::dart::collision::CollisionDetector;
using ::dart::dynamics::DegreeOfFreedom;
using ::r3::ompl::DARTGeometricStateSpace;

DARTGeometricStateSpace::DARTGeometricStateSpace(
        std::vector<DegreeOfFreedom *> const &dofs,
        std::vector<double> const &weights,
        CollisionDetector *collision_detector)
    : dofs_(dofs)
    , is_circular_(dofs.size())
    , collision_detector_(collision_detector)
{
    using ::boost::make_shared;
    using ::ompl::base::RealVectorStateSpace;
    using ::ompl::base::SO2StateSpace;

    BOOST_ASSERT(dofs.size() == weights.size());

    for (size_t idof = 0; idof < dofs.size(); ++idof) {
        DegreeOfFreedom *dof = dofs[idof];
        double const &weight = weights[idof];

        // Construct a one-dimensional state space for each DOF. Add them, in
        // the same order as "dofs", to this CompoundStateSpace.
        bool &is_circular = is_circular_[idof];
        is_circular = IsDOFCircular(dof);

        if (is_circular) {
            addSubspace(make_shared<SO2StateSpace>(), weight);
        } else {
            auto const state_space = make_shared<RealVectorStateSpace>(1);
            state_space->setName(dof->getName());
            state_space->setBounds(
                dof->getPositionLowerLimit(),
                dof->getPositionUpperLimit()
            );
            addSubspace(state_space, weight);
        }

        // Build a list of all Skeleton's influenced by these joints.
        skeletons_.insert(dof->getSkeleton());
    }
}

void DARTGeometricStateSpace::ApplyState(StateType const *state)
{
    using ::dart::dynamics::Skeleton;

    typedef ::ompl::base::SO2StateSpace::StateType SO2State;
    typedef ::ompl::base::RealVectorStateSpace::StateType RealVectorState;

    // Update joint values.
    for (size_t idof = 0; idof < dofs_.size(); ++idof) {
        double value;

        if (is_circular_[idof]) {
            value = (*state)[idof]->as<SO2State>()->value;
        } else {
            value = (*state)[idof]->as<RealVectorState>()->values[0];
        }

        dofs_[idof]->setPosition(value);
    }

    // Compute forward kinematics.
    for (Skeleton *skeleton : skeletons_) {
        skeleton->computeForwardKinematics(true, false, false);
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
    // TODO: How do we tell if a DOF is circular?
    return false;
}

