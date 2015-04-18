#include <vector>
#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <boost/container/vector.hpp>
#include <dart/collision/collision.h>
#include <dart/dynamics/dynamics.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

using ::dart::dynamics::DegreeOfFreedom;
using ::ompl::base::StateSpacePtr;

class DARTGeometricStateSpace;
typedef boost::shared_ptr<DARTGeometricStateSpace> DARTGeometricStateSpacePtr;

class DARTGeometricStateSpace : ::ompl::base::CompoundStateSpace {
public:
    typedef ::ompl::base::CompoundStateSpace::StateType StateType;

    DARTGeometricStateSpace(
        ::std::vector<DegreeOfFreedom *> const &dofs,
        ::std::vector<double> const &weights,
        ::dart::collision::CollisionDetector *collision_detector);

    void ApplyState(StateType const *state);
    bool IsInCollision();

private:
    ::std::vector<DegreeOfFreedom *> dofs_;
    ::boost::container::vector<bool> is_circular_;
    ::boost::unordered_set<::dart::dynamics::Skeleton *> skeletons_;
    ::dart::collision::CollisionDetector *collision_detector_;

    static bool IsDOFCircular(DegreeOfFreedom const *dof);
};


DARTGeometricStateSpace::DARTGeometricStateSpace(
        std::vector<DegreeOfFreedom *> const &dofs,
        std::vector<double> const &weights,
        ::dart::collision::CollisionDetector *collision_detector)
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
    return collision_detector_->detectCollision(false, false);
}

bool DARTGeometricStateSpace::IsDOFCircular(DegreeOfFreedom const *dof)
{
    // TODO: How do we tell if a DOF is circular?
    return false;
}

// ---

class DARTGeometricStateValidityChecker : ::ompl::base::StateValidityChecker {
public:
    typedef ::ompl::base::State State;

    DARTGeometricStateValidityChecker(
        ::ompl::base::SpaceInformation *space_info);
    DARTGeometricStateValidityChecker(
        ::ompl::base::SpaceInformationPtr const &space_info);

    virtual bool isValid(State const *state) const;
    virtual double clearance(State const *state) const;
    virtual double clearance(State const *state, State *validState,
                             bool &validStateAvailable) const;

private:
    DARTGeometricStateSpace *state_space_;
};


DARTGeometricStateValidityChecker::DARTGeometricStateValidityChecker(
        ::ompl::base::SpaceInformation *space_info)
    : ::ompl::base::StateValidityChecker(space_info)
    //, state_space_(space_info->getStateSpace()->as<DARTGeometricStateSpace>())
{
}

DARTGeometricStateValidityChecker::DARTGeometricStateValidityChecker(
        ::ompl::base::SpaceInformationPtr const &space_info)
    : ::ompl::base::StateValidityChecker(space_info)
    //, state_space_(space_info->getStateSpace()->as<DARTGeometricStateSpace>())
{
}

bool DARTGeometricStateValidityChecker::isValid(State const *state) const
{
    typedef DARTGeometricStateSpace::StateType DARTState;

    state_space_->ApplyState(state->as<DARTState>());
    return !state_space_->IsInCollision();
}

// ---

